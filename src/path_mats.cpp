#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Li{0.0f};
        Color3f t{1.0f};

        Ray3f recursiveRay = ray;
        Intersection xo;
        float successProbability;
        
        while (true) {
            if (!scene->rayIntersect(recursiveRay, xo)) {
                break;
            }

            // Contribution from material sampling
            if (xo.mesh->isEmitter()) {
                EmitterQueryRecord emitterRecord{recursiveRay.o, xo.p, xo.shFrame.n};
                emitterRecord.uv = xo.uv;
                Li += t * xo.mesh->getEmitter()->eval(emitterRecord);       // Li += t * Le(x0)
            }

            // Russian Roulette
            successProbability = std::min(t.maxCoeff(), 0.99f);
            if (sampler->next1D() > successProbability || successProbability == 0.0f) {
                break;
            }

            t /= successProbability;

            BSDFQueryRecord bsdfRecord{xo.shFrame.toLocal(-recursiveRay.d)};
            bsdfRecord.uv = xo.uv;
            t *= xo.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

            recursiveRay = Ray3f{xo.p, xo.shFrame.toWorld(bsdfRecord.wo)};
        }

        return Li;
    }

    std::string toString() const {
        return "PathMatsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END
