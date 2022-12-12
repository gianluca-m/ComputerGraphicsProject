#include <nori/emitter.h>
#include <nori/scene.h>

NORI_NAMESPACE_BEGIN

class DirectionalLight : public Emitter {
public:
    DirectionalLight(const PropertyList &props) {
        m_power = props.getColor("power");
        m_direction = props.getVector3("direction");

        /* Hardcoded values because the Preprocess function is not working atm*/
        m_worldCenter = Point3f(0,0,0);
        /* Radius of 300 is an overapproximation as mentioned in the book. An exact measure would not be worth the cost/outcome*/
        m_worldRadius = 300;
    }

    /* This function is not working as it should*/
    virtual void Preprocess(const Scene *scene) {
        auto bbox = scene->getBoundingBox();
        m_worldCenter = bbox.getCenter();
        m_worldRadius = (bbox.max - m_worldCenter).norm();
    }
   

    virtual Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const {
        lRec.wi = -m_direction;
        lRec.pdf = 1.f;

        //lRec.ref stores the reference point known as "p"
        //From there we take 2 steps of radius such that we are guaranteed to land outside of our circle.
        auto shadowRayLength = 2*m_worldRadius;
        lRec.shadowRay = Ray3f(lRec.ref,lRec.wi, Epsilon, shadowRayLength);

        return m_power;
    }

    virtual Color3f eval(const EmitterQueryRecord &lRec) const {
        return m_power * M_PI * pow(m_worldRadius,2.0f);
    }

    virtual float pdf(const EmitterQueryRecord &lRec) const {
        return 0.f;
    }

    std::string toString() const {
        return tfm::format(
                "DirectionalLight[\n"
                "  power = %s,\n"
                //"  position = %f,\n"
                "  direction = %s,\n"
                "  center = %s\n"
                "  radius = %s\n"
                "]",
                m_power,
                //m_position.toString(),
                m_direction.toString(),
                m_worldCenter.toString(),
                m_worldRadius
            );
    }

private:
    Color3f m_power;
    //Point3f m_position;
    Vector3f m_direction;
    Point3f m_worldCenter;
    float m_worldRadius;
};

NORI_REGISTER_CLASS(DirectionalLight, "directional");
NORI_NAMESPACE_END
