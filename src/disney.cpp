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
        m_roughness = propList.getFloat("roughness", 0.f);
        m_sheen = propList.getFloat("sheen", 0.f);
        m_clearcoat = propList.getFloat("clearcoat", 0.f);
        m_specularTint = propList.getFloat("specularTint", 0.f);
    }

    static float SchlickWeight(float cosTheta) {
        float m = clamp(1 - cosTheta, 0.f, 1.f);
        return pow(m,5);
    }

    static float FrSchlick(float R0, float cosTheta) {
        return lerp(SchlickWeight(cosTheta), R0, 1);
    }

    static float GTR1(float cosTheta, float alpha){
        float a = alpha * alpha;
        return (a - 1) / (M_PI * log(a) * (1 + (a-1) * cosTheta * cosTheta));
    }
    static float smithGGGX(float cosTheta, float alpha) {
        float a = alpha * alpha;
        float c = cosTheta * cosTheta;
        return 1 / (cosTheta + sqrt(a + c - a * c));
    } 
/*
    Color3f Diffuse(const float wo, const float wi, const float cosThetaD, Color3f baseColor) const{
        float Fo = SchlickWeight(wo);
        float Fi = SchlickWeight(wi);
        float FD90 = (.5f + 2 * m_roughness * pow(cosThetaD,2)) - 1.0f;
        return baseColor * INV_PI * (1.f + Fo * FD90) * (1 + Fi * FD90);
    }

    Color3f Sheen(const float cosThetaD, const Vector3f &h, Color3f baseColor) const{
        if(h.x() == 0 && h.y() == 0 && h.z() == 0) return 0;
        return baseColor * SchlickWeight(cosThetaD);
    }

   Color3f Specular(const float cosThetaD,const float cosThetaL,const float cosThetaV,Color3f specularColor) const {
        float a = std::max(0.001f, m_roughness, m_roughness);
        float Ds = 0;
        float Fh = SchlickWeight(cosThetaD);
        float Gs = smithGGGX(cosThetaL, a) * smithGGGX(cosThetaV, a);
        return lerpColor(specularColor, 1, Fh)* Gs * Ds;
    }
 

    Color3f Clearcoat(const Vector3f h, float cosThetaL , float cosThetaV, float cosThetaD)const{
        if(h.x() == 0 && h.y() == 0 && h.z() == 0) return 0;

        float alpha = std::max(0.001f, m_roughness * m_roughness);
        float Dr = GTR1(Frame::cosTheta(h), alpha);
        float Fr = FrSchlick(.04f, cosThetaD);
        float Gr = smithGGGX(cosThetaL, 0.25f) * smithGGGX(cosThetaV,0.25f);

        return m_clearcoat * Gr * Fr * Dr / 4.f;
    }
*/
    virtual Color3f eval(const BSDFQueryRecord &bRec) const override {
        auto v = bRec.wi;
        auto l = bRec.wo;
        auto h = (v + l).normalized();

        auto cosThetaV = Frame::cosTheta(v);
        auto cosThetaL = Frame::cosTheta(l);
        auto cosThetaH = Frame::cosTheta(h);
        auto cosThetaD = l.dot(h);

        if (cosThetaV <= 0 || cosThetaL <= 0 || bRec.measure != ESolidAngle) {
            return 0;
        }

        auto baseColor = m_albedo -> eval(bRec.uv);
        auto L = baseColor.getLuminance();
        auto tint = Color3f(0);

        if(L <= 0){
            tint = Color3f(1);
        } else {
            tint = Color3f(baseColor / L);
        }

        //auto specC = lerpColor(m_specular*0.08f * lerpColor(1, tint, m_specularTint), baseColor, m_metallic);
        /*
        auto diffuseColor = Diffuse(cosThetaL,cosThetaV,cosThetaD,baseColor);
        auto sheenColor = Sheen(cosThetaD, h, baseColor);
        auto specularColor = Specular(cosThetaD, cosThetaL, cosThetaV, specC);
        auto clearcoatColor = Clearcoat(h,cosThetaL,cosThetaV,cosThetaD);

        return (1.f - m_metallic) * diffuseColor + specularColor + clearcoatColor + sheenColor;
        */
       return 0;
    }

    virtual float pdf(const BSDFQueryRecord &bRec) const override {
        auto v = bRec.wi;
        auto l = bRec.wo;
        auto h = (v + l).normalized();

        auto cosTheta_v = Frame::cosTheta(v);
        auto cosTheta_l = Frame::cosTheta(l);
        auto cosTheta_h = Frame::cosTheta(h);

        if (cosTheta_l <= 0 || cosTheta_v <= 0 || bRec.measure != ESolidAngle) {
            return 0;
        }

        auto alpha =pow(m_roughness,2);
        auto Jh = 1 / (4 * h.dot(l));
        auto Ds = 0; //Warp::squareToGTR2Pdf(h, alpha);

        return (1 - m_metallic) * cosTheta_l * INV_PI
                + m_metallic * Ds * cosTheta_h * Jh;
    }

    virtual Color3f sample(BSDFQueryRecord &bRec, const Point2f &sample) const override {
        bRec.measure = EDiscrete;

        return Color3f{1.0f};
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
            "]",
            m_albedo, m_specular,m_specularTint, m_metallic,m_roughness,m_sheen,m_clearcoat);
    }
private:
    float m_specular,m_specularTint, m_metallic,m_roughness,m_sheen,m_clearcoat;
    Texture<Color3f> * m_albedo;
};

NORI_REGISTER_CLASS(DisneyBSDF, "disneyBSDF");
NORI_NAMESPACE_END
