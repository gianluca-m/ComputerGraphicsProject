#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class HeterogeneousMedium : public Medium {
public:
    HeterogeneousMedium(const PropertyList &props) {
        m_sigma_a = props.getColor("sigma_a", Color3f{1.0f});
        m_sigma_s = props.getColor("sigma_s", Color3f{1.0f});
        m_sigma_t = m_sigma_a + m_sigma_s;
        m_albedo = m_sigma_s / m_sigma_t;

        m_density_type = props.getInteger("density_type", 0);

        // TODO (gimoro): find max_density if not constant
        m_max_density = props.getFloat("max_density", 1.0f);
        m_inv_max_density = 1.0f / m_max_density;

        // Exponential density
        m_up_dir = props.getVector3("up_dir", Vector3f{0.0f, 0.0f, 1.0f}).normalized();
        m_exp_a = std::min(1.0f, props.getFloat("exp_a", m_max_density));
        m_exp_b = props.getFloat("exp_b", 2.0f);

        Vector3f size = props.getVector3("size", Vector3f{1.0f}).cwiseAbs();
        Vector3f center = props.getVector3("center", Vector3f{0.0f});
        m_bbox = BoundingBox3f(center - size, center + size);
    }

    Color3f Tr(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            return Color3f{1.0f};
        }

        float t = 0.0f;
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
            tr *= 1.0f - curr_density * m_max_density;      // density / (1 / max_density) = density * max_density
        }
        
        return tr;
    }

    Color3f sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            return Color3f{1.0f};
        }

        float t = 0.0f;
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
            if (sampler->next1D() < curr_density * m_max_density) {      // density / (1 / max_density) = density * max_density
                mRec.hasInteraction = true;
                mRec.p = ray(t);
                return m_albedo * curr_density;
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
                "  density_type = %i,\n"
                "  phasefunction = [ %s ],\n"
                "  bbox = %s\n"
                "]",
                m_sigma_a.toString(), m_sigma_s.toString(), m_density_type, 
                m_phasefunction->toString(), m_bbox.toString());
    }

private:
    float m_inv_max_density;
    int m_density_type;     // const, exp, noise function, volume grid

    // used for exponential density
    Vector3f m_up_dir;      
    float m_exp_a;
    float m_exp_b;

    float getDensity(const Point3f &p) const {
        if (!m_bbox.contains(p)) return 0.0f;
        
        switch (m_density_type) {
            case 0:     // const
                return m_max_density;
            case 1:     // exp
                return exponentialDensity(p);                
            case 2:     // noise function
                throw NoriException("Not implemented");
            case 3:     // volume grid
                throw NoriException("Not implemented");
            
            default:
                throw NoriException("HeterogeneousMedium: Undefined density type");
        }
    }

    float exponentialDensity(const Point3f &p) const {
        float h = (p - m_bbox.min).dot(m_up_dir);
        return m_exp_a * exp(-m_exp_b * h);
    }
};

NORI_REGISTER_CLASS(HeterogeneousMedium, "heterogeneous")
NORI_NAMESPACE_END
