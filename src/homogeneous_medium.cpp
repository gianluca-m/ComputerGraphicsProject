#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium(const PropertyList &props) {
        m_sigma_a = props.getColor("sigma_a", Color3f{1.0f});
        m_sigma_s = props.getColor("sigma_s", Color3f{1.0f});
        m_sigma_t = m_sigma_a + m_sigma_s;
        m_albedo = m_sigma_s / m_sigma_t;
        m_max_density = props.getFloat("max_density", 1.0f);

        Vector3f size = props.getVector3("size", Vector3f{1.0f}).cwiseAbs();
        Vector3f center = props.getVector3("center", Vector3f{0.0f});
        m_bbox = BoundingBox3f(center - size, center + size);
    }

    Color3f Tr(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking even for homogeneous because other approach produced weird results
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            return Color3f{1.0f};
        }

        float t = 0.0f;
        float tMax = std::min(mRec.tMax, farT);
        Color3f tr{1.0f};
        float curr_density;
        float densityDivSigmaT = m_max_density / m_sigma_t.maxCoeff();

        while (true) {
            t += -log(1.0f - sampler->next1D()) * densityDivSigmaT;

            if (t >= tMax) {
                break;
            }
                
            curr_density = getDensity(ray(t));
            tr *= 1.0f - curr_density * m_max_density;
        }
        
        return tr;
    }

    Color3f sample(const Ray3f &ray, Sampler *sampler, MediumQueryRecord &mRec) const override {
        // Delta tracking even for homogeneous because other approach produced weird results
        float nearT, farT;
        if (!rayIntersect(ray, nearT, farT)) {
            return Color3f{1.0f};
        }

        float t = 0.0f;
        float tMax = std::min(mRec.tMax, farT);
        float curr_density;
        float densityDivSigmaT = m_max_density / m_sigma_t.maxCoeff();

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

    float getDensity(const Point3f &p) const {
        return m_bbox.contains(p) ? m_max_density : 0.0f;
    }

    bool rayIntersect(const Ray3f &ray, float &nearT, float &farT) const override {
        return m_bbox.rayIntersect(ray, nearT, farT);
    }

    void addChild(NoriObject *obj) {
        switch (obj->getClassType()) {
            case EPhaseFunction:
                if (m_phasefunction) 
                    throw NoriException("HomogeneousMedium: There is already a phase function defined!");
                m_phasefunction = static_cast<PhaseFunction*>(obj);
                break;

            default:
                throw NoriException("HomogeneousMedium::addChild(<%s>) is not supported!",
                                    classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const override {
        return tfm::format(
                "HomogeneousMedium[\n"
                "  sigma_a = %s,\n"
                "  sigma_s = %s,\n"
                "  phasefunction = %s,\n"
                "  bbox = %s\n"
                "]",
                m_sigma_a.toString(), m_sigma_s.toString(), m_phasefunction->toString(), m_bbox.toString());
    }
};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous")
NORI_NAMESPACE_END
