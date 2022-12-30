#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class HomogeneousMedium : public Medium {
public:
    HomogeneousMedium(const PropertyList &props) {
        m_density_scale = props.getFloat("density_scale", 1.0f);
        m_sigma_a = props.getColor("sigma_a", Color3f{1.0f});
        m_sigma_s = props.getColor("sigma_s", Color3f{1.0f});
        m_sigma_t = m_sigma_a + m_sigma_s;
        m_sigma_t *= m_density_scale;
        m_albedo = m_sigma_s / m_sigma_t;
        m_max_density = props.getFloat("max_density", 1.0f);
        if (m_max_density == 0.0f) throw NoriException("HeterogeneousMedium: max_density cannot be 0");
        m_inv_max_density = 1.0f / m_max_density;

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
        // Delta tracking even for homogeneous because other approach produced weird results
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
            t += -log(1.0f - sampler->next1D()) * densityDivSigmaT;

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
                "HeterogeneousMedium[\n"
                "  sigma_a = %s,\n"
                "  sigma_s = %s,\n"
                "  sigma_t = %s,\n"
                "  albedo  = %s,\n"
                "  max_density = %f,\n"
                "  inv_max_density = %f,\n"
                "  density_scale = %f,\n"
                "  phasefunction = [ %s ],\n"
                "  bbox = %s\n"
                "]",
                m_sigma_a.toString(), m_sigma_s.toString(),
                m_sigma_t.toString(), m_albedo.toString(),
                m_max_density, m_inv_max_density, m_density_scale,
                m_phasefunction->toString(), m_bbox.toString());
    }

private:
    float getDensity(const Point3f &p) const {
        return m_bbox.contains(p) ? m_max_density : 0.0f;
    }
};

NORI_REGISTER_CLASS(HomogeneousMedium, "homogeneous")
NORI_NAMESPACE_END
