#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectEmsIntegrator : public Integrator {
public:
    DirectEmsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        Color3f Lo{0.0f};

        for (auto light : scene->getLights()) {
            EmitterQueryRecord emitterRecord{its.p};
            emitterRecord.uv = its.uv;
            auto LeDivPdf = light->sample(emitterRecord, sampler->next2D());       // Le / pdf_em

            if (scene->rayIntersect(emitterRecord.shadowRay)) continue;

            auto wi = its.shFrame.toLocal(emitterRecord.wi);

            BSDFQueryRecord bsdfRecord{its.shFrame.toLocal(-ray.d), wi, ESolidAngle};
            bsdfRecord.p = its.p;
            bsdfRecord.uv = its.uv;

            auto shadowRay = emitterRecord.shadowRay.d;
            auto cosTheta = Frame::cosTheta(wi);

            Lo += its.mesh->getBSDF()->eval(bsdfRecord) * LeDivPdf * cosTheta;
        }

        if (its.mesh->isEmitter()) {
            EmitterQueryRecord emitterRecord{ray.o, its.p, its.shFrame.n};
            emitterRecord.uv = its.uv;
            Lo += its.mesh->getEmitter()->eval(emitterRecord);   // add Le
        }

        return Lo;
    }

    std::string toString() const {
        return "DirectEmsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectEmsIntegrator, "direct_ems");
NORI_NAMESPACE_END
