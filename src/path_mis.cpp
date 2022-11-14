#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Li{0.0f};
        Color3f t{1.0f};

        Ray3f recursiveRay = ray;
        Intersection xo;
        float successProbability;

        auto wMat = 1.0f;
        auto wEm = 0.0f;
        
        while (true) {
            if (!scene->rayIntersect(recursiveRay, xo)) {
                break;
            }

            // Contribution from material sampling
            if (xo.mesh->isEmitter()) {
                EmitterQueryRecord emitterRecord{recursiveRay.o, xo.p, xo.shFrame.n};
                Li += t * wMat * xo.mesh->getEmitter()->eval(emitterRecord);        // Li += t * w_mat * Le(x0)
            }

            // Russian Roulette
            successProbability = std::min(t.x(), 0.99f);
            if (sampler->next1D() > successProbability || successProbability == 0.0f) {
                break;
            }

            t /= successProbability;

            // Contribution from emitter sampling
            auto randomEmitter = scene->getRandomEmitter(sampler->next1D());
            EmitterQueryRecord emitterRecord{xo.p};
            Color3f LeDivPdf = randomEmitter->sample(emitterRecord, sampler->next2D()) * (float) scene->getLights().size();
            if (!scene->rayIntersect(emitterRecord.shadowRay)) {
                auto wi = xo.shFrame.toLocal(-recursiveRay.d);
                auto wo = xo.shFrame.toLocal(emitterRecord.wi);
                auto cosTheta = Frame::cosTheta(wo);

                BSDFQueryRecord bsdfRecord{wi, wo, ESolidAngle};
                bsdfRecord.p = xo.p;
                bsdfRecord.uv = xo.uv;
                
                auto pdfEm = randomEmitter->pdf(emitterRecord);
                auto pdfMat = xo.mesh->getBSDF()->pdf(bsdfRecord);
                auto pdfSum = pdfEm + pdfMat;
                wEm = (pdfSum > 0) ? pdfEm / pdfSum : 0.0f;

                Li += wEm * t * (xo.mesh->getBSDF()->eval(bsdfRecord) * LeDivPdf * cosTheta);
            }

            BSDFQueryRecord bsdfRecord{xo.shFrame.toLocal(-recursiveRay.d)};
            bsdfRecord.uv = xo.uv;
            t *= xo.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

            recursiveRay = Ray3f{xo.p, xo.shFrame.toWorld(bsdfRecord.wo)};

            // update wMat
            if (bsdfRecord.measure == EDiscrete) {
                wMat = 1.0f;
            }
            else {
                Intersection its;
                if (!scene->rayIntersect(recursiveRay, its)) break;

                if (its.mesh->isEmitter()) {
                    EmitterQueryRecord eRecord{xo.p, its.p, its.shFrame.n};
                    auto pdfEm = its.mesh->getEmitter()->pdf(eRecord);
                    auto pdfMat = xo.mesh->getBSDF()->pdf(bsdfRecord);
                    auto pdfSum = pdfEm + pdfMat;
                    wMat = (pdfSum > 0) ? pdfMat / pdfSum : 0.0f;
                }
            }
        }

        return Li;
    }

    std::string toString() const {
        return "PathMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END
