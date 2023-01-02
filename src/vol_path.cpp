#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class VolumetricPathIntegrator : public Integrator {
public:
    VolumetricPathIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Color3f Li{0.0f};
        Color3f t{1.0f};

        float n = (float) scene->getLights().size();

        Ray3f recursiveRay = ray;
        Intersection its;
        float successProbability;
        auto wMat = 1.0f;
        auto wEm = 0.0f;

        bool sceneIntersection = scene->rayIntersect(recursiveRay, its);
        auto allMedia = scene->getMedia();
        
        while (true) {
            auto medium = scene->getRandomMedium(recursiveRay, sampler->next1D());

            // Sample free path
            float tmax = sceneIntersection ? (its.p - recursiveRay.o).norm() : recursiveRay.maxt;
            MediumQueryRecord mediumRecord(tmax);
            Color3f albedo{1.0f};
            if (medium != nullptr) {
                albedo = medium->sample(recursiveRay, sampler, mediumRecord);
            }

            if (mediumRecord.hasInteraction) {      // Volume interaction
                // Russian Roulette
                successProbability = std::min(t.maxCoeff(), 0.99f);
                if (sampler->next1D() > successProbability) {
                    break;
                }
                t /= successProbability;
                t *= albedo;

                Vector3f wo;
                auto pdfMat = medium->getPhasefunction()->sample(wo, sampler->next2D());

                auto randomEmitter = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord emitterRecord{mediumRecord.p};
                // No need to set emitterRecord.uv here specifically, because it will be set in Emitter::sample()
                Color3f LeDivPdf = randomEmitter->sample(emitterRecord, sampler->next2D()) * n;
                if (!scene->rayIntersect(emitterRecord.shadowRay)) {
                    MediumQueryRecord shadowRayMediumRecord(emitterRecord.shadowRay.maxt);
                    Color3f Tr{1.0f};
                    for (auto m : allMedia) {
                        Tr *= m->Tr(emitterRecord.shadowRay, sampler, shadowRayMediumRecord);
                    }

                    auto pdfEm = randomEmitter->pdf(emitterRecord);
                    auto pdfSum = pdfEm + pdfMat;
                    wEm = (pdfSum > 0) ? pdfEm / pdfSum : 0.0f;

                    Li += t * Tr * LeDivPdf * pdfMat;
                }

                recursiveRay = Ray3f{mediumRecord.p, wo};

                sceneIntersection = scene->rayIntersect(recursiveRay, its);
                if (sceneIntersection && its.mesh->isEmitter()) {
                    EmitterQueryRecord eRecord{recursiveRay.o, its.p, its.shFrame.n};
                    eRecord.uv = its.uv;
                    auto pdfEm = its.mesh->getEmitter()->pdf(eRecord);
                    auto pdfSum = pdfEm + pdfMat;
                    wMat = (pdfSum > 0) ? pdfMat / pdfSum : 0.0f;
                }
            }
            else if (!sceneIntersection) {      // No medium interaction and no surface interaction
                break;
            }
            else {  // Surface Interaction
                // Contribution from material sampling
                if (its.mesh->isEmitter()) {
                    EmitterQueryRecord emitterRecord{recursiveRay.o, its.p, its.shFrame.n};
                    emitterRecord.uv = its.uv;
                    Color3f Tr{1.0f};
                    for (auto m : allMedia) {
                        Tr *= m->Tr(recursiveRay, sampler, mediumRecord);
                    }

                    Li += t * wMat * its.mesh->getEmitter()->eval(emitterRecord) * Tr;
                }

                // Russian Roulette
                successProbability = std::min(t.maxCoeff(), 0.99f);
                if (sampler->next1D() > successProbability) {
                    break;
                }
                t /= successProbability;

                // Contribution from emitter sampling
                auto randomEmitter = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord emitterRecord{its.p};
                // No need to set emitterRecord.uv here specifically, because it will be set in Emitter::sample()
                Color3f LeDivPdf = randomEmitter->sample(emitterRecord, sampler->next2D()) * n;
                if (!scene->rayIntersect(emitterRecord.shadowRay)) {
                    auto wi = its.shFrame.toLocal(-recursiveRay.d);
                    auto wo = its.shFrame.toLocal(emitterRecord.wi);
                    auto cosTheta = Frame::cosTheta(wo);

                    BSDFQueryRecord bsdfRecord{wi, wo, ESolidAngle};
                    bsdfRecord.p = its.p;
                    bsdfRecord.uv = its.uv;
                    
                    auto pdfEm = randomEmitter->pdf(emitterRecord);
                    auto pdfMat = its.mesh->getBSDF()->pdf(bsdfRecord);
                    auto pdfSum = pdfEm + pdfMat;
                    wEm = (pdfSum > 0) ? pdfEm / pdfSum : 0.0f;

                    Color3f Tr{1.0f};
                    MediumQueryRecord shadowRayMediumRecord(emitterRecord.shadowRay.maxt);
                    for (auto m : allMedia) {
                        Tr *= m->Tr(emitterRecord.shadowRay, sampler, shadowRayMediumRecord);
                    } 

                    Li += wEm * t * its.mesh->getBSDF()->eval(bsdfRecord) * LeDivPdf * cosTheta * Tr;
                }

                BSDFQueryRecord bsdfRecord{its.shFrame.toLocal(-recursiveRay.d)};
                bsdfRecord.uv = its.uv;
                t *= its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

                recursiveRay = Ray3f{its.p, its.shFrame.toWorld(bsdfRecord.wo)};

                // update wMat
                sceneIntersection = scene->rayIntersect(recursiveRay, its);
                if (bsdfRecord.measure == EDiscrete) {
                    wMat = 1.0f;
                }
                else if (sceneIntersection && its.mesh->isEmitter()) {
                    EmitterQueryRecord eRecord{recursiveRay.o, its.p, its.shFrame.n};
                    eRecord.uv = its.uv;
                    auto pdfEm = its.mesh->getEmitter()->pdf(eRecord);
                    auto pdfMat = its.mesh->getBSDF()->pdf(bsdfRecord);
                    auto pdfSum = pdfEm + pdfMat;
                    wMat = (pdfSum > 0) ? pdfMat / pdfSum : 0.0f; 
                }
            }
        }

        return Li;
    }

    std::string toString() const {
        return "VolumetricPathIntegrator[]";
    }
};

NORI_REGISTER_CLASS(VolumetricPathIntegrator, "vol_path");
NORI_NAMESPACE_END
