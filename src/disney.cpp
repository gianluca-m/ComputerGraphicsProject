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

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN
using namespace std;

class DisneyBSDF : public BSDF {
public:
    DisneyBSDF(const PropertyList &propList) {
        PropertyList l;
        l.setColor("value", propList.getColor("albedo"));
        m_albedo = static_cast<Texture<Color3f> *>(NoriObjectFactory::createInstance("constant_color", l));
        
        m_specular = propList.getFloat("specular", 0.f);
        m_metallic = propList.getFloat("metallic", 0.f);
        m_roughness = propList.getFloat("roughness", 0.0f);
        m_sheen = propList.getFloat("sheen", 0.f);
        m_clearcoat = propList.getFloat("clearcoat", 0.f);
        m_specularTint = propList.getFloat("specularTint", 0.0f);
        m_clearcoatGloss = propList.getFloat("clearcoatGloss", 0.1f);

        // The original value for this is 0.25 (original Disney)
        // But the higher the value to more visible is the clearcoat effect!
        m_clearcoatIntensity = 1.5; 

        m_sheenIntensity = 1.0;
    }

    static float SchlickWeight(float cosTheta) {
        float m = clamp(1 - cosTheta, 0.f, 1.f);
        return powf(m,5);
    }

    static float FrSchlick(float R0, float cosTheta) {
        return lerp(SchlickWeight(cosTheta), R0, 1);
    }

    static float smithGGGX(float cosTheta, float alpha) {
        float a2 = alpha * alpha;
        float c = cosTheta * cosTheta;
        return 1.f / (cosTheta + sqrt(a2 + c - a2 * c));
    } 

    static Color3f lerpColor(float t, const Color3f v1, const Color3f v2) {
        return (1 - t) * v1 + t * v2;
    }

    Color3f Diffuse(const float wo, const float wi, const float cosThetaD, Color3f baseColor) const{
        float Fo = SchlickWeight(wo);
        float Fi = SchlickWeight(wi);
        float FD90 = (0.5f + 2 * m_roughness * powf(cosThetaD,2));
        return baseColor * INV_PI * (1.f + (FD90-1.f)*Fo)*(1.f + (FD90 -1.f) * Fi);
    }

    Color3f Sheen(const float cosThetaD, Color3f baseColor) const{
        return m_sheen * lerpColor(0.1f,Color3f(1),baseColor) * SchlickWeight(cosThetaD) * m_sheenIntensity;
    }

   Color3f Specular(const float cosThetaD,const float cosThetaL,const float cosThetaV,Color3f specularColor, const Vector3f h) const {
        float a = std::max(0.001f, m_roughness* m_roughness);
        float Ds = Warp::squareToGTR2Pdf(h,a);
        float Fh = SchlickWeight(cosThetaD);
        float Gs = smithGGGX(cosThetaL, a) * smithGGGX(cosThetaV, a);
        return lerpColor(Fh, specularColor,Color3f(1))* Gs * Ds;
    }
 

    Color3f Clearcoat(const Vector3f h, float cosThetaL , float cosThetaV, float cosThetaD)const{
        float Dr = Warp::squareToGTR1Pdf(h, lerp(m_clearcoatGloss, 0.1f, 0.001f)); 
        float Fr = FrSchlick(0.04f, cosThetaD);
        float Gr = smithGGGX(cosThetaL, 0.25f) * smithGGGX(cosThetaV,0.25f);

        return (m_clearcoat * Gr * Fr * Dr) * m_clearcoatIntensity;
    }

    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
        auto v = bRec.wi;
        auto l = bRec.wo;
        auto h = (v + l).normalized();

        auto cosThetaV = Frame::cosTheta(v);
        auto cosThetaL = Frame::cosTheta(l);
        auto cosThetaH = Frame::cosTheta(h);
        auto cosThetaD = l.dot(h);

        if (cosThetaV < 0.f || cosThetaL < 0.f)return 0;

        auto baseColor = m_albedo -> eval(bRec.uv);
        auto L = baseColor.getLuminance();
        auto tint = Color3f(0);

        if(L <= 0){
            tint = Color3f(1);
        } else {
            tint = Color3f(baseColor / L);
        }

        auto specC = lerpColor(m_metallic,m_specular *0.08f * lerpColor(m_specularTint, Color3f(1), tint), baseColor);
        
        auto diffuseColor = Diffuse(cosThetaL,cosThetaV,cosThetaD,baseColor);
        auto sheenColor = Sheen(cosThetaD,tint);
        auto specularColor = Specular(cosThetaD, cosThetaL, cosThetaV, specC, h);
        auto clearcoatColor = Clearcoat(h,cosThetaL,cosThetaV,cosThetaD);

        return (1.f - m_metallic) * diffuseColor + specularColor + clearcoatColor + sheenColor;
    }

    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        auto v = bRec.wi;
        auto l = bRec.wo;
        auto h = (v + l).normalized();

        auto cosThetaL = Frame::cosTheta(l);
        auto cosThetaH = Frame::cosTheta(h);
        auto cosThetaD = l.dot(h);

        if (cosThetaL <= 0)return 0;

        auto diffuseWeight = (1.f - m_metallic) * 0.5f;

        auto GTR2Weight = 1.f / (1.f + m_clearcoat);
        auto GTR1Weight = (1.f - GTR2Weight);

        auto alpha = max(0.001f, powf(m_roughness,2));

        // Use abs value to prevent NaN and invalid radiance!!!
        auto Jh = 1.f / (4 * abs(cosThetaD));

        auto GTR2PDF = Warp::squareToGTR2Pdf(h, alpha);
        auto GTR1PDF = Warp::squareToGTR1Pdf(h, lerp(m_clearcoatGloss,0.1f,0.001f)); 

        return diffuseWeight * cosThetaL * INV_PI + (1.f - diffuseWeight)* (GTR2Weight * GTR2PDF * Jh * cosThetaH + (GTR1Weight * GTR1PDF * Jh * cosThetaH));
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {

        auto v = bRec.wi;
        auto cosThetaV = Frame::cosTheta(v);
        Vector3f h;

        if(cosThetaV <= 0.f)return 0;

        auto diffuseWeight = (1.f - m_metallic) * 0.5f;

        if(sample.x() < diffuseWeight){
            bRec.wo = Warp::squareToCosineHemisphere(Point2f(sample.x() / diffuseWeight, sample.y()));
        } else {
            auto newX = (sample.x() - diffuseWeight) / (1 - diffuseWeight);
            auto GTR2Weight = 1.f / (1.f + m_clearcoat);

            if(newX < GTR2Weight){
                auto alpha = max(0.001f, m_roughness*m_roughness);
                newX /= GTR2Weight;
                h = Warp::squareToGTR2(Point2f(newX, sample.y()), alpha);
            } else {
                newX = (newX - GTR2Weight) / (1.f - GTR2Weight);
                h = Warp::squareToGTR1(Point2f(newX,sample.y()), lerp(m_clearcoatGloss,0.1f,0.001f));
            }
            bRec.wo = (2 * v.dot(h) * h - v).normalized();
        }
        auto cosTheta = Frame::cosTheta(bRec.wo);
        if(cosTheta <= 0.f)return 0;

        return eval(bRec) * cosTheta / pdf(bRec);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "DisneyBSDF[\n"
            "  albedo = %f,\n"
            "  specular = %f\n"
            "  specularTint = %f\n"
            "  metallic = %f,\n"
            "  roughness = %f\n"
            "  sheen = %f,\n"
            "  clearcoat = %f\n"
            "  clearcoatGloss = %f\n"
            "]",
            m_albedo, m_specular,m_specularTint, m_metallic,m_roughness,m_sheen,m_clearcoat,m_clearcoatGloss);
    }
private:
    float m_specular,m_specularTint, m_metallic,m_roughness,m_sheen,m_clearcoat,m_clearcoatGloss,m_clearcoatIntensity, m_sheenIntensity;

    Texture<Color3f> * m_albedo;
};


NORI_REGISTER_CLASS(DisneyBSDF, "disneyBSDF");
NORI_NAMESPACE_END
