/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

	

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

        auto numDepositedPhotons = 0;
        m_emittedPhotonCount = 0;

		while (numDepositedPhotons < m_photonCount) {
            Ray3f recursiveRay;
            Intersection xi;
            float successProbability;

            auto randomEmitter = scene->getRandomEmitter(sampler->next1D());
            Color3f W = randomEmitter->samplePhoton(recursiveRay, sampler->next2D(), sampler->next2D()) * (float) scene->getLights().size();

            while (true) {
                if (!scene->rayIntersect(recursiveRay, xi)) break;

                if (xi.mesh->getBSDF()->isDiffuse()) {
                    m_photonMap->push_back(Photon{xi.p, -recursiveRay.d, W});
                    numDepositedPhotons++;
                }

                // Russian roulette
                successProbability = std::min(W.maxCoeff(), 0.99f);
                if (sampler->next1D() > successProbability || successProbability == 0.0f) {
                    break;
                }

                W /= successProbability;

                BSDFQueryRecord bsdfRecord{xi.shFrame.toLocal(-recursiveRay.d)};
                bsdfRecord.uv = xi.uv;
                W *= xi.mesh->getBSDF()->sample(bsdfRecord, sampler->next2D());

                recursiveRay = Ray3f{xi.p, xi.shFrame.toWorld(bsdfRecord.wo)};
            }
            
            m_emittedPhotonCount++;
        }

		/* Build the photon map */
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		Color3f Li{0.0f};
        Color3f t{1.0f};

        Ray3f recursiveRay = ray;
        Intersection xo;
        float successProbability;

        while (true) {
            if (!scene->rayIntersect(recursiveRay, xo)) break;

            // Contribution from material sampling
            if (xo.mesh->isEmitter()) {
                EmitterQueryRecord emitterRecord{recursiveRay.o, xo.p, xo.shFrame.n};
                emitterRecord.uv = xo.uv;
                Li += t * xo.mesh->getEmitter()->eval(emitterRecord);       // Li += t * Le(x0)
            }

            if (xo.mesh->getBSDF()->isDiffuse()) {
                Color3f photonDensityEstimate{0.0f};

                std::vector<uint32_t> photons;
                m_photonMap->search(xo.p, m_photonRadius, photons);

                for (auto i : photons) {
                    const Photon &photon = (*m_photonMap)[i];

                    BSDFQueryRecord bsdfRecord{xo.shFrame.toLocal(-recursiveRay.d), xo.shFrame.toLocal(photon.getDirection()), ESolidAngle};
                    bsdfRecord.uv = xo.uv;

                    photonDensityEstimate += xo.mesh->getBSDF()->eval(bsdfRecord) * photon.getPower();
                }

                photonDensityEstimate /= M_PI * m_emittedPhotonCount * powf(m_photonRadius, 2);
                Li += t * photonDensityEstimate;
                break;
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

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    int m_photonCount;
    int m_emittedPhotonCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
