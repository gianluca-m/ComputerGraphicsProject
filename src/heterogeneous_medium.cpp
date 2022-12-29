#include <nori/medium.h>
#include <filesystem/resolver.h>
#include <unordered_map>
#define NANOVDB_USE_ZIP 1
#include <nanovdb/util/IO.h>
#include <nori/perlinnoise.h>

#define KEY(x, y, z) (x + y * 10000 + z * 10000000)

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
        m_density_type = props.getInteger("density_type", 0);

        Vector3f size = props.getVector3("size", Vector3f{1.0f}).cwiseAbs();
        Vector3f center = props.getVector3("center", Vector3f{0.0f});
        m_bbox = BoundingBox3f(center - size, center + size);


        if (m_density_type == 1) {  // Exponential density
            m_up_dir = props.getVector3("up_dir", Vector3f{0.0f, 0.0f, 1.0f}).normalized();
            m_exp_b = props.getFloat("exp_b", 2.0f);
        } 
        else if (m_density_type == 2) {    // Volume grid 
            auto filename = getFileResolver()->resolve(props.getString("volume_grid")).str();
            auto handle = nanovdb::io::readGrid(filename);
            nanovdb::FloatGrid* density_grid = nullptr;

            for (uint32_t i = 0; i < handle.gridCount(); i++) {
                auto *curr_grid = handle.grid<float>(i);
                if (strcmp(curr_grid->gridName(), "density") == 0) {
                    density_grid = curr_grid;
                    break;
                }
            }

            if (density_grid == nullptr) throw NoriException("HeterogeneousMedium: No density grid found in '%s'", filename);

            auto grid_bbox = density_grid->worldBBox();
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
            auto accessor = density_grid->getAccessor();
            float curr_density;
            for (int x = (int) std::ceil(grid_bbox_min.x()); x < grid_bbox_max.x(); x++) {
                for (int y = (int) std::ceil(grid_bbox_min.y()); y < grid_bbox_max.y(); y++) {
                    for (int z = (int) std::ceil(grid_bbox_min.z()); z < grid_bbox_max.z(); z++) {
                        curr_density = accessor.getValue(nanovdb::Coord(x, y, z));

                        if (curr_density < 0) throw NoriException("HeterogeneousMedium: Found illegal negative density value in grid!");

                        m_max_density = std::max(m_max_density, curr_density);

                        // using hashmap, because using grid does not work in getGridDensity() for some reason...
                        // This unfortunately introduces some additional preprocessing time.
                        m_density_grid_map[KEY(x, y, z)] = curr_density;
                    }
                }
            }
        } 
        else if (m_density_type == 3) {     // perlin noise sphere
            m_center = Point3f{center};
            m_radius = props.getFloat("radius", 1.0f);
            m_frequency = props.getFloat("frequency", 3.5f);
            m_bbox = BoundingBox3f(center - Vector3f{m_radius}, center + Vector3f{m_radius});
        }

        if (m_max_density == 0.0f) throw NoriException("HeterogeneousMedium: max_density cannot be 0");

        m_inv_max_density = 1.0f / m_max_density;
        m_bbox_size = m_bbox.max - m_bbox.min;
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
        float curr_density;
        float densityDivSigmaT = m_inv_max_density / m_sigma_t.maxCoeff();

        while (true) {
            t += -log(1.0f - sampler->next1D()) * densityDivSigmaT;

            if (t >= tMax) {
                break;
            }
                
            curr_density = getDensity(ray(t));
            tr *= 1.0f - std::max(0.0f, curr_density * m_inv_max_density);
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
        float curr_density;
        float densityDivSigmaT = m_inv_max_density / m_sigma_t.maxCoeff();

        while (true) {
            t += -log(1.0f - sampler->next1D()) * densityDivSigmaT;

            if (t >= tMax) {
                mRec.hasInteraction = false;
                break;
            }

            curr_density = getDensity(ray(t));
            if (sampler->next1D() < curr_density * m_inv_max_density) {
                mRec.hasInteraction = true;
                mRec.p = ray(t);
                return m_albedo;
            }
        }
        
        return Color3f{1.f};
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
                m_max_density, m_inv_max_density, m_density_type, m_density_scale,
                m_phasefunction->toString(), m_bbox.toString());
    }


private:
    float m_inv_max_density;
    int m_density_type;     // const, exp, volume grid

    // used for exponential density
    Vector3f m_up_dir;
    float m_exp_b;

    // used for volume grid
    std::unordered_map<uint32_t, float> m_density_grid_map;
    BoundingBox3f m_density_grid_bbox;
    Vector3f m_density_grid_bbox_size;
    Point3f m_bbox_size;

    // used for perlin noise sphere
    Point3f m_center;
    float m_radius;
    float m_frequency;


    float getDensity(const Point3f &p) const {
        if (!m_bbox.contains(p)) return 0.0f;
        
        switch (m_density_type) {
            case 0:     // const
                return m_max_density;
            case 1:     // exp
                return getExponentialDensity(p);
            case 2:     // volume grid
                return getGridDensity(p);
            case 3:     // perlin noise sphere
                return getPerlinNoiseDensity(p);
            
            default:
                throw NoriException("HeterogeneousMedium: Undefined density type");
        }
    }

    float getExponentialDensity(const Point3f &p) const {
        float h = (p - m_bbox.min).dot(m_up_dir);
        return m_max_density * exp(-m_exp_b * h);
    }

    float getGridDensity(const Point3f &p) const {
        Point3f relative_position = (p - m_bbox.min).cwiseQuotient(m_bbox_size);

        int x = (int) round(m_density_grid_bbox.min.x() + relative_position.x() * m_density_grid_bbox_size.x());
        int y = (int) round(m_density_grid_bbox.min.y() + relative_position.y() * m_density_grid_bbox_size.y());
        int z = (int) round(m_density_grid_bbox.min.z() + relative_position.z() * m_density_grid_bbox_size.z());

        auto element = m_density_grid_map.find(KEY(x, y, z));
        return element != m_density_grid_map.end() ? element->second : 0.0f;
    }

    float getPerlinNoiseDensity(const Point3f &p) const {
        auto dist = (p - m_center).norm();

        if (dist > m_radius) return 0.0f;       // outside of sphere

        if (dist == 0.0f) dist = Epsilon;

        Point3f relative_position = (p - m_bbox.min).cwiseQuotient(m_bbox_size);
        relative_position *= m_frequency * (m_radius / dist);

        return PerlinNoise::get3DPerlinNoise(relative_position);
    }
};

NORI_REGISTER_CLASS(HeterogeneousMedium, "heterogeneous")
NORI_NAMESPACE_END
