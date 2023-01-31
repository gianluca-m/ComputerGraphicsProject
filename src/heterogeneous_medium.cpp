#include <nori/medium.h>
#include <filesystem/resolver.h>
#include <unordered_map>
#define NANOVDB_USE_ZIP 1
#include <nanovdb/util/IO.h>
#include <nori/perlinnoise.h>

// Possible density types
#define EXPONENTIAL 0
#define VOLUME_GRID 1
#define PERLIN_NOISE 2
#define EXP_SPHERE_LAYER 3

NORI_NAMESPACE_BEGIN

class HeterogeneousMedium : public Medium {
public:
    HeterogeneousMedium(const PropertyList &props) {
        m_density_scale = props.getFloat("density_scale", 1.0f);
        m_sigma_a = props.getColor("sigma_a", Color3f{1.0f});
        m_sigma_s = props.getColor("sigma_s", Color3f{1.0f});
        m_sigma_t = m_sigma_a + m_sigma_s;
        m_sigma_t *= m_density_scale;
        m_albedo = m_sigma_s / m_sigma_t;

        m_max_density = props.getFloat("max_density", 1.0f);
        m_density_type = props.getInteger("density_type", EXPONENTIAL);

        Vector3f size = props.getVector3("size", Vector3f{1.0f}).cwiseAbs();
        Vector3f center = props.getVector3("center", Vector3f{0.0f});

        m_bbox.expandBy(center - size);
        m_bbox.expandBy(center + size);

        // The transform "toWorld" should only contain rotations!
        // The center of the volume should be defined using the "center" property!
        Eigen::Matrix4f translate_center;
        translate_center << 1, 0, 0, center.x(),
                            0, 1, 0, center.y(),
                            0, 0, 1, center.z(),
                            0, 0, 0, 1;
        Transform transform = props.getTransform("toWorld", Transform()) * Transform{translate_center};
        m_inv_transform = transform.getInverseMatrix();

        Point3f origin{0.0f};
        Point3f bbox_min = transform * Point3f(origin - size);
        Point3f bbox_max = transform * Point3f(origin + size);
        m_bbox.expandBy(bbox_min);
        m_bbox.expandBy(bbox_max);


        if (m_density_type == EXPONENTIAL) {    // Exponential density
            m_up_dir = props.getVector3("up_dir", Vector3f{0.0f, 0.0f, 1.0f}).normalized();
            m_exp_b = props.getFloat("exp_b", 2.0f);
        } 
        else if (m_density_type == VOLUME_GRID) {    // Volume grid 
            m_non_transformed_bbox_min = origin - size;
            m_non_transformed_bbox_size = 2 * size;

            auto filename = getFileResolver()->resolve(props.getString("volume_grid")).str();
            m_handle = nanovdb::io::readGrid(filename);
            m_density_grid = nullptr;

            for (uint32_t i = 0; i < m_handle.gridCount(); i++) {
                auto *curr_grid = m_handle.grid<float>(i);
                if (strcmp(curr_grid->gridName(), "density") == 0) {
                    m_density_grid = curr_grid;
                    break;
                }
            }

            if (m_density_grid == nullptr) throw NoriException("HeterogeneousMedium: No density grid found in '%s'", filename);

            auto grid_bbox = m_density_grid->indexBBox();
            Point3f grid_bbox_min{
                (float) grid_bbox.min()[0],
                (float) grid_bbox.min()[1],
                (float) grid_bbox.min()[2]
            };
            Point3f grid_bbox_max{
                (float) grid_bbox.max()[0],
                (float) grid_bbox.max()[1],
                (float) grid_bbox.max()[2]
            };

            cout << "density_grid_bbox: min=" << grid_bbox_min.toString() << ", max=" << grid_bbox_max.toString() << endl;

            m_density_grid_bbox = BoundingBox3f{grid_bbox_min, grid_bbox_max};
            m_density_grid_bbox_size = m_density_grid_bbox.max - m_density_grid_bbox.min;

            m_max_density = 0.0f;
            auto accessor = m_density_grid->getAccessor();
            float curr_density;
            for (int x = (int) std::ceil(grid_bbox_min.x()); x < grid_bbox_max.x(); x++) {
                for (int y = (int) std::ceil(grid_bbox_min.y()); y < grid_bbox_max.y(); y++) {
                    for (int z = (int) std::ceil(grid_bbox_min.z()); z < grid_bbox_max.z(); z++) {
                        curr_density = accessor.getValue(nanovdb::Coord(x, y, z));

                        if (curr_density < 0) throw NoriException("HeterogeneousMedium: Found illegal negative density value in grid!");

                        m_max_density = std::max(m_max_density, curr_density);
                    }
                }
            }
        } 
        else if (m_density_type == PERLIN_NOISE) {     // perlin noise shape
            // Implemented by Eric
            m_center = Point3f{center};
            m_is_sphere = props.getBoolean("isSphere", true);   // Defines if cube or sphere
            m_radius = props.getFloat("radius", 1.0f);
            m_frequency = props.getFloat("frequency", 3.5f);
            m_bbox = BoundingBox3f(center - Vector3f{m_radius}, center + Vector3f{m_radius});
            m_bbox_size = m_bbox.max - m_bbox.min;
        }
        else if (m_density_type == EXP_SPHERE_LAYER) {     // exponential sphere layer
            m_center = Point3f{center};
            m_min_radius = props.getFloat("min_radius", 0.5f);
            m_max_radius = props.getFloat("max_radius", 0.5f);
            m_exp_b = props.getFloat("exp_b", 2.0f);
            m_bbox = BoundingBox3f(center - Vector3f{m_max_radius}, center + Vector3f{m_max_radius});
        }

        if (m_max_density == 0.0f) throw NoriException("HeterogeneousMedium: max_density cannot be 0");
        m_inv_max_density = 1.0f / m_max_density;
    }

    Color3f Tr(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            return Color3f{1.0f};
        }

        float t = std::max(0.0f, nearT);
        float tMax = std::min(mRec.tMax, farT);
        Color3f tr{1.0f};
        float densityDivSigmaT = m_inv_max_density / m_sigma_t.maxCoeff();

        while (true) {
            t -= log(1.0f - sampler->next1D()) * densityDivSigmaT;

            if (t >= tMax) {
                break;
            }
                
            tr *= 1.0f - std::max(0.0f, getDensity(ray(t)) * m_inv_max_density);
        }
        
        return tr;
    }

    Color3f sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            mRec.hasInteraction = false;
            return Color3f{1.0f};
        }

        float t = std::max(0.0f, nearT);
        float tMax = std::min(mRec.tMax, farT);

        // I don't know how this can even happen, but it happens...
        if (tMax == std::numeric_limits<float>::infinity()) {
            mRec.hasInteraction = false;
            return Color3f{1.0f};
        }

        float densityDivSigmaT = m_inv_max_density / m_sigma_t.maxCoeff();

        while (true) {
            t -= log(1.0f - sampler->next1D()) * densityDivSigmaT;

            if (t >= tMax) {
                mRec.hasInteraction = false;
                break;
            }

            if (sampler->next1D() < getDensity(ray(t)) * m_inv_max_density) {
                mRec.hasInteraction = true;
                mRec.p = ray(t);
                return m_albedo;
            }
        }
        
        return Color3f{1.0f};
    }

    bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const override {
        return m_bbox.rayIntersect(ray, nearT, farT);
    }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case EPhaseFunction:
                if (m_phasefunction) 
                    throw NoriException("HeterogeneousMedium: There is already a phase function defined!");
                m_phasefunction = static_cast<PhaseFunction*>(obj);
                break;

            default:
                throw NoriException("HeterogeneousMedium::addChild(<%s>) is not supported!",
                                    classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const override {
        std::string density_type_str;
        switch (m_density_type) {
            case EXPONENTIAL:
                density_type_str = "exponential";
                break;
            case VOLUME_GRID:
                density_type_str = "volume grid";
                break;
            case PERLIN_NOISE:
                density_type_str = "perlin noise";
                break;
            case EXP_SPHERE_LAYER:
                density_type_str = "exponential sphere layer";
                break;
            default:
                density_type_str = std::to_string(m_density_type);
                break;
        }

        return tfm::format(
                "HeterogeneousMedium[\n"
                "  sigma_a = %s,\n"
                "  sigma_s = %s,\n"
                "  sigma_t = %s,\n"
                "  albedo  = %s,\n"
                "  density_type = %i,\n"
                "  max_density = %f,\n"
                "  inv_max_density = %f,\n"
                "  density_scale = %f,\n"
                "  phasefunction = [ %s ],\n"
                "  bbox = %s\n"
                "]",
                m_sigma_a.toString(), m_sigma_s.toString(),
                m_sigma_t.toString(), m_albedo.toString(),
                density_type_str, m_max_density, m_inv_max_density, m_density_scale,
                m_phasefunction->toString(), m_bbox.toString());
    }


private:
    int m_density_type;

    // used for exponential density
    Vector3f m_up_dir;
    float m_exp_b;

    // used for volume grid
    BoundingBox3f m_density_grid_bbox;
    Vector3f m_density_grid_bbox_size;
    Point3f m_non_transformed_bbox_min;
    Point3f m_non_transformed_bbox_size;
    nanovdb::FloatGrid* m_density_grid;
    // For some reason, NanoVDB getValue in getGridDensity() only works if I leave the handle as a class property...
    nanovdb::GridHandle<nanovdb::HostBuffer> m_handle;

    // used to transform back positions inside the volume
    Transform m_inv_transform;

    // used for perlin noise shape
    // Implemented by Eric
    Point3f m_center;
    Point3f m_bbox_size;
    float m_radius;
    float m_frequency;
    bool m_is_sphere;

    // used for exponential sphere layer (--> Earth Atmosphere in final image)
    float m_max_radius;
    float m_min_radius;


    float getDensity(const Point3f &p) const {
        if (!m_bbox.contains(p)) return 0.0f;       
        
        switch (m_density_type) {
            case EXPONENTIAL:     // exp
                return getExponentialDensity(p);
            case VOLUME_GRID:     // volume grid
                return getGridDensity(p);
            case PERLIN_NOISE:     // perlin noise shape
                return getPerlinNoiseDensity(p);
            case EXP_SPHERE_LAYER:
                return getExponentialDensitySphereLayer(p);
            
            default:
                throw NoriException("HeterogeneousMedium: Undefined density type");
        }
    }

    float getExponentialDensity(const Point3f &p) const {
        float h = (p - m_bbox.min).dot(m_up_dir);
        return m_max_density * exp(-m_exp_b * h);
    }

    float getExponentialDensitySphereLayer(const Point3f &p) const {
        auto dist = (p - m_center).norm();
        if (dist > m_max_radius || dist < m_min_radius) return 0.0f;       // not within sphere layer

        Vector3f up_dir = (p - m_center).normalized();
        float h = (p - m_center).dot(up_dir) - m_min_radius;
        return m_max_density * exp(-m_exp_b * h);
    }

    float getGridDensity(const Point3f &p) const {
        // Transform point back so it's aligned with the m_density_grid_bbox
        // because m_density_grid_bbox is not transformed
        Point3f back_transformed_p = m_inv_transform * p;
        Point3f relative_position = (back_transformed_p - m_non_transformed_bbox_min).cwiseQuotient(m_non_transformed_bbox_size);

        int x = (int) round(m_density_grid_bbox.min.x() + relative_position.x() * m_density_grid_bbox_size.x());
        int y = (int) round(m_density_grid_bbox.min.y() + relative_position.y() * m_density_grid_bbox_size.y());
        int z = (int) round(m_density_grid_bbox.min.z() + relative_position.z() * m_density_grid_bbox_size.z());

        return m_density_grid->tree().getValue(nanovdb::Coord(x, y, z));
    }

    // Implemented by Eric
    float getPerlinNoiseDensity(const Point3f &p) const {
        auto dist = (p - m_center).norm();

        if (m_is_sphere && dist > m_radius) return 0.0f;     // Outside of sphere

        if (dist == 0.0f) dist = Epsilon;

        Point3f relative_position = (p - m_bbox.min).cwiseQuotient(m_bbox_size);
        relative_position *= m_frequency * (m_radius / dist);

        return PerlinNoise::get3DPerlinNoise(relative_position);
    }
};

NORI_REGISTER_CLASS(HeterogeneousMedium, "heterogeneous")
NORI_NAMESPACE_END
