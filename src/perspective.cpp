/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

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

#include <nori/camera.h>
#include <nori/rfilter.h>
#include <nori/warp.h>
#include <Eigen/Geometry>

using namespace std;
NORI_NAMESPACE_BEGIN

/**
 * \brief Perspective camera with depth of field
 *
 * This class implements a simple perspective camera model. It uses an
 * infinitesimally small aperture, creating an infinite depth of field.
 */
class PerspectiveCamera : public Camera {
public:
    PerspectiveCamera(const PropertyList &propList) {
        /* Width and height in pixels. Default: 720p */
        m_outputSize.x() = propList.getInteger("width", 1280);
        m_outputSize.y() = propList.getInteger("height", 720);
        m_invOutputSize = m_outputSize.cast<float>().cwiseInverse();

        /* Specifies an optional camera-to-world transformation. Default: none */
        m_cameraToWorld = propList.getTransform("toWorld", Transform());

        /* Horizontal field of view in degrees */
        m_fov = propList.getFloat("fov", 30.0f);

        /* Near and far clipping planes in world-space units */
        m_nearClip = propList.getFloat("nearClip", 1e-4f);
        m_farClip = propList.getFloat("farClip", 1e4f);

        m_focalLength = propList.getFloat("focalLength", 0.f);
        m_lensRadius = propList.getFloat("lensRadius", 0.f);

        

        m_bokeh = propList.getBoolean("bokeh",false);

        m_distortion = propList.getBoolean("distortion", false);
        K1 = propList.getFloat("K1", 0.0f);
        K2 = propList.getFloat("K2", 0.0f);

        m_motionblur = propList.getBoolean("motionblur", false);
        m_finalMotion = propList.getTransform("motion", Transform());
        

        m_rfilter = NULL;
    }

    

    virtual void activate() override {
        float aspect = m_outputSize.x() / (float) m_outputSize.y();

        /* Project vectors in camera space onto a plane at z=1:
         *
         *  xProj = cot * x / z
         *  yProj = cot * y / z
         *  zProj = (far * (z - near)) / (z * (far-near))
         *  The cotangent factor ensures that the field of view is 
         *  mapped to the interval [-1, 1].
         */
        float recip = 1.0f / (m_farClip - m_nearClip),
              cot = 1.0f / std::tan(degToRad(m_fov / 2.0f));

        Eigen::Matrix4f perspective;
        perspective <<
            cot, 0,   0,   0,
            0, cot,   0,   0,
            0,   0,   m_farClip * recip, -m_nearClip * m_farClip * recip,
            0,   0,   1,   0;

        /**
         * Translation and scaling to shift the clip coordinates into the
         * range from zero to one. Also takes the aspect ratio into account.
         */
        m_sampleToCamera = Transform( 
            Eigen::DiagonalMatrix<float, 3>(Vector3f(0.5f, -0.5f * aspect, 1.0f)) *
            Eigen::Translation<float, 3>(1.0f, -1.0f/aspect, 0.0f) * perspective).inverse();

        /* If no reconstruction filter was assigned, instantiate a Gaussian filter */
        if (!m_rfilter) {
            m_rfilter = static_cast<ReconstructionFilter *>(
                    NoriObjectFactory::createInstance("gaussian", PropertyList()));
            m_rfilter->activate();
        }
    }

    Color3f sampleRay(Ray3f &ray,
            const Point2f &samplePosition,
            const Point2f &apertureSample,
            float contribution) const {
        /* Compute the corresponding position on the 
           near plane (in local camera space) */
        Point3f nearP = m_sampleToCamera * Point3f(
            samplePosition.x() * m_invOutputSize.x(),
            samplePosition.y() * m_invOutputSize.y(), 0.0f);

        if(m_distortion){
            //change nearP
            float distort = distortionFunction(Vector2f(nearP.x() / nearP.z(), nearP.y() / nearP.z()).norm());
            nearP.x() *= distort;
            nearP.y() *= distort;
        }
        /* Turn into a normalized ray direction, and
           adjust the ray interval accordingly */

        ray.o = Point3f(0, 0, 0);
        ray.d = nearP.normalized(); 

        /* Depth of Field PBRT 6.2.3*/
        if(m_lensRadius > 0 ) {
            Point2f pLens;
            if(m_bokeh){
                pLens = m_lensRadius * bokehShift(apertureSample);
            } else {
                pLens = m_lensRadius* Warp::squareToUniformDisk(apertureSample);
            }
            float ft = m_focalLength / ray.d.z();
            Point3f pFocus = ray(ft); //Ray cast just like in the book works

            ray.o = Point3f(pLens.x(), pLens.y(), 0);
            ray.d = (pFocus - ray.o).normalized();
            
        }

        float invZ = 1.0f / ray.d.z();

        if(m_motionblur){
            Transform trans(m_cameraToWorld.getMatrix() * (1.0f - contribution) + m_finalMotion.getMatrix() * contribution);
            //Use trans instead of m_cameraToWorld
            ray.o = trans * ray.o;
            ray.d = trans * ray.d;
        } else {
            ray.o = m_cameraToWorld * ray.o;
            ray.d = m_cameraToWorld * ray.d;
        }

        ray.mint = m_nearClip * invZ;
        ray.maxt = m_farClip * invZ;
        ray.update();

        return Color3f(1.0f);
    }


    Point2f bokehShift(Point2f sample) const {
        //Take an offset step for the sampling method instead of the direct coordinates
        Point2f offset = 2.f * sample - Vector2f(1, 1);
      
        auto x = offset.x();
        auto y = offset.y();

        if (x == 0 && y == 0) return Point2f(0, 0);

        // Concentric Disk mapping 
        // http://l2program.co.uk/900/concentric-disk-sampling
        float theta, r;
        if (abs(x) > abs(y)) {
            r = x;
            theta = INV_FOURPI * (y / x);
        } else {
            r = y;
            theta = INV_TWOPI - INV_FOURPI * (x / y);
        }
        //Polar Coordinates
        return r * Point2f(cos(theta), sin(theta));
    }

    //https://en.wikipedia.org/wiki/Distortion_(optics)
    float distortionFunction(float r) const{
        //The further away from the focus point, the larger is r
        if(r == 0)return 1.f;
        auto fin = (1.0f + K1 * powf(r,2) + K2*powf(r,4));

        return fin;
        
    }

    virtual void addChild(NoriObject *obj) override {
        switch (obj->getClassType()) {
            case EReconstructionFilter:
                if (m_rfilter)
                    throw NoriException("Camera: tried to register multiple reconstruction filters!");
                m_rfilter = static_cast<ReconstructionFilter *>(obj);
                break;

            default:
                throw NoriException("Camera::addChild(<%s>) is not supported!",
                    classTypeName(obj->getClassType()));
        }
    }

    /// Return a human-readable summary
    virtual std::string toString() const override {
        return tfm::format(
            "PerspectiveCamera[\n"
            "  cameraToWorld = %s,\n"
            "  outputSize = %s,\n"
            "  fov = %f,\n"
            "  clip = [%f, %f],\n"
            "  rfilter = %s\n"
            "]",
            indent(m_cameraToWorld.toString(), 18),
            m_outputSize.toString(),
            m_fov,
            m_nearClip,
            m_farClip,
            indent(m_rfilter->toString())
        );
    }
private:
    Vector2f m_invOutputSize;
    Transform m_sampleToCamera;
    Transform m_cameraToWorld;
    float m_fov;
    float m_nearClip;
    float m_farClip;
    float m_focalLength;
    float m_lensRadius; //=f-value
    bool m_distortion;
    bool m_bokeh;
    float K1;
    float K2;
    bool m_motionblur;
    Transform m_finalMotion;
};

NORI_REGISTER_CLASS(PerspectiveCamera, "perspective");
NORI_NAMESPACE_END
