#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        auto normal = its.shFrame.n;
        Color3f finalColor{0.0f};
        Vector2f sample;

        for (auto light : scene->getLights()) {
            EmitterQueryRecord emitterRecord{its.p};
            Color3f incidentRadiance = light->sample(emitterRecord, sample);

            if (scene->rayIntersect(emitterRecord.shadowRay)) {
                continue;
            }

            BSDFQueryRecord bsdfRecord{its.shFrame.toLocal(emitterRecord.wi), its.shFrame.toLocal(-ray.d), ESolidAngle};
            bsdfRecord.p = its.p;
            bsdfRecord.uv = its.uv;

            auto shadowRay = emitterRecord.shadowRay.d;
            auto cosTheta = normal.dot(shadowRay);

            finalColor += its.mesh->getBSDF()->eval(bsdfRecord) * incidentRadiance * abs(cosTheta);
        }

        // finalColor = 1/N * sum((f(p, wo, wi) * L(p,wi) * |cos(theta_i)|) / p(wi))
        //  note: scale by 1/N done by nori
        return finalColor;
    }

    std::string toString() const {
        return "DirectIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END
