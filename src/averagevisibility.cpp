#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class AverageVisibilityIntegrator : public Integrator {
public:
    AverageVisibilityIntegrator(const PropertyList &props) {
        m_rayLength = props.getFloat("length");
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(1.0f);


        /*  Using the intersection point its.p, the world space shading normal its.shFrame.n, 
            and the provided sampler, generate a point on the hemisphere and trace a ray into this direction.
            The ray should have a user-specifiable length
        */
        Ray3f randomRay{its.p, Warp::sampleUniformHemisphere(sampler, its.shFrame.n), Epsilon, m_rayLength};
        
        if (scene->rayIntersect(randomRay)) {
            return Color3f(0.0f);
        }
        
        return Color3f(1.0f);
    }

    std::string toString() const {
        return "AverageVisibilityIntegrator[]";
    }

private:
    float m_rayLength;
};

NORI_REGISTER_CLASS(AverageVisibilityIntegrator, "av");
NORI_NAMESPACE_END
