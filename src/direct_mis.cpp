#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMisIntegrator : public Integrator {
public:
    DirectMisIntegrator(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);

        auto wo = its.shFrame.toLocal(-ray.d);

        // Le
        Color3f Le{0.0f};
        if (its.mesh->isEmitter()) {
            EmitterQueryRecord emitterRec{ray.o, its.p, its.shFrame.n};
            Le = its.mesh->getEmitter()->eval(emitterRec);
        }


        // Em
        Color3f Lem{0.0f};
        for (auto light : scene->getLights()) {
            EmitterQueryRecord emitterRecord{its.p};
            auto LeDivPdf = light->sample(emitterRecord, sampler->next2D());       // Le / pdf_em

            if (scene->rayIntersect(emitterRecord.shadowRay)) continue;

            auto wi = its.shFrame.toLocal(emitterRecord.wi);

            auto cosTheta = Frame::cosTheta(wi);
            BSDFQueryRecord bsdfRecord{wo, wi, ESolidAngle};
            bsdfRecord.p = its.p;
            bsdfRecord.uv = its.uv;
            auto F = its.mesh->getBSDF()->eval(bsdfRecord) * LeDivPdf * cosTheta;

            auto pdfEm = light->pdf(emitterRecord);
            auto pdfMat = its.mesh->getBSDF()->pdf(bsdfRecord);
            auto pdfSum = pdfEm + pdfMat;
            auto wEm = (pdfSum > 0) ? pdfEm / pdfSum : 0.0f;

            Lem += wEm * F;
        }


        // Mat
        BSDFQueryRecord bsdfRecord{wo};
        bsdfRecord.uv = its.uv;
        auto fr = its.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());        // (fr * cos(theta)) / pdf_mat

        Color3f Li{0.0f};
        auto pdfEm = 0.0f;
        Ray3f wi{its.p, its.shFrame.toWorld(bsdfRecord.wo)};
        Intersection wiIts;
        if (scene->rayIntersect(wi, wiIts) && wiIts.mesh->isEmitter()) {
            EmitterQueryRecord emitterRecord{its.p, wiIts.p, wiIts.shFrame.n};
            Li = wiIts.mesh->getEmitter()->eval(emitterRecord);
            pdfEm = wiIts.mesh->getEmitter()->pdf(emitterRecord);
        }

        auto pdfMat = its.mesh->getBSDF()->pdf(bsdfRecord);
        auto pdfSum = pdfEm + pdfMat;
        auto wMat = (pdfSum > 0) ? pdfMat / pdfSum : 0.0f;

        Color3f Lmat{wMat * Li * fr};

        return Le + Lem + Lmat;
    }

    std::string toString() const {
        return "DirectMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectMisIntegrator, "direct_mis");
NORI_NAMESPACE_END
