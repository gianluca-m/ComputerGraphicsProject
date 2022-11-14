#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter {
public:
    PointLight(const PropertyList &props) {
        m_power = props.getColor("power");
        m_position = props.getPoint3("position");
    }

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
        lRec.wi = (m_position - lRec.ref).normalized();
        lRec.pdf = pdf(lRec);
        lRec.p = m_position;

        auto shadowRayLength = (m_position - lRec.ref).norm();
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, shadowRayLength);
        
        // phi / (4 * pi * r^2) / pdf
        return eval(lRec) / pdf(lRec);
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        auto shadowRayLength = (m_position - lRec.ref).squaredNorm();
        return m_power / (4.0f * M_PI * shadowRayLength);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        return 1.0f;
    }

    std::string toString() const {
        return "PointLight[]";
    }

private:
    Color3f m_power;
    Point3f m_position;
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END
