#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMatsIntegrator : public Integrator {
public:
    DirectMatsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        BSDFQueryRecord bsdfRecord{its.shFrame.toLocal(-ray.d)};
        bsdfRecord.uv = its.uv;
        Color3f fr = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());        // (fr * cos(theta)) / pdf

        Color3f Li{0.0f};
        Ray3f wi{its.p, its.shFrame.toWorld(bsdfRecord.wo)};
        Intersection wiIts;
        if (scene->rayIntersect(wi, wiIts) && wiIts.mesh->isEmitter()) {
            EmitterQueryRecord emitterRecord{its.p, wiIts.p, wiIts.shFrame.n};
            Li = wiIts.mesh->getEmitter()->eval(emitterRecord);
        }

        Color3f Le{0.0f};
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord emitterRec{ray.o, its.p, its.shFrame.n};
            Le = its.mesh->getEmitter()->eval(emitterRec);   // add Le
        }

        return Le + Li * fr;
    }

    std::string toString() const {
        return "DirectMatsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectMatsIntegrator, "direct_mats");
NORI_NAMESPACE_END
