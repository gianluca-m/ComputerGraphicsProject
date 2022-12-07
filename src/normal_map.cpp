/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

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

#include <nori/object.h>
#include <nori/texture.h>
#include <filesystem/resolver.h>
#include <nori/shape.h>
#include "stb_image.h"

NORI_NAMESPACE_BEGIN

class NormalMap : public Texture<Vector3f> {
public:
    NormalMap(const PropertyList &props) {
        m_filename = getFileResolver()->resolve(props.getString("filename")).str();
        m_scale = props.getVector2("scale", Vector2f(1.0f));
        m_shift = props.getVector2("shift", Vector2f(0.0f));

        m_image = stbi_load(m_filename.c_str(), &m_width, &m_height, &m_channels, 0);

        if (m_image == NULL) {
            tfm::printfln("Error: Loading normal map '%s' failed\n\t%s", m_filename, stbi_failure_reason());
        }
    }

    std::string toString() const override {
        return tfm::format(
            "NormalMap[\n"
                "  filename = \"%s\",\n"
                "  scale = %s,\n"
                "  shift = %s\n"
                "]",
            m_filename, m_scale.toString(), m_shift.toString());
    }

    Vector3f eval(const Point2f &uv) const override {
        // Map uv to texture coordinates
        auto mappedX = uv.x() * m_width;
        auto mappedY = (1.0f - uv.y()) * m_height;

        // Apply scale and shift
        auto x = ((int) (mappedX * m_scale.x() + m_shift.x() * m_width)) % m_width;
        auto y = ((int) (mappedY * m_scale.y() + m_shift.y() * m_height)) % m_height;

        int index = (x + y * m_width) * m_channels;
        auto r = (float) (m_image[index] / 255.0);
        auto g = (float) (m_image[index + 1] / 255.0);
        auto b = (float) (m_image[index + 2] / 255.0);

        return (2.0f * Vector3f{r, g, b} - Vector3f{1.0f}).normalized();
    }

    void applyNormal(const Point2f &uv, const Vector3f &tangent, const Vector3f &bitangent, Intersection &its) override {
        Vector3f normal = this->eval(uv);
        Frame frame{tangent, bitangent, its.geoFrame.n};
        its.shFrame.n = frame.toWorld(normal);
    }


protected:
    std::string m_filename;
    unsigned char *m_image;
    int m_height;
    int m_width;
    int m_channels;
    Vector2f m_scale;
    Vector2f m_shift;
};

NORI_REGISTER_CLASS(NormalMap, "normal_map")
NORI_NAMESPACE_END
