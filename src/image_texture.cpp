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
#include "stb_image.h"

NORI_NAMESPACE_BEGIN

class ImageTexture : public Texture<Color3f> {
public:
    ImageTexture(const PropertyList &props) {
        m_filename = getFileResolver()->resolve(props.getString("filename")).str();
        m_scale = props.getVector2("scale", Vector2f(1.0f));
        m_shift = props.getVector2("shift", Vector2f(0.0f));
        m_linear_rgb = props.getBoolean("linear_rgb", true);

        m_image = stbi_load(m_filename.c_str(), &m_width, &m_height, &m_channels, 0);

        if (m_image == NULL) {
            tfm::printfln("Error: Loading image texture '%s' failed\n\t%s", m_filename, stbi_failure_reason());
        }
        else {
            tfm::printfln("Loading image texture successful. #channels = %i, width = %i, height = %i", m_channels, m_width, m_height);
        }
    }

    std::string toString() const {
        return tfm::format(
            "ImageTexture[\n"
                "  filename = \"%s\",\n"
                "  scale = %s,\n"
                "  shift = %s,\n"
                "  linear_rgb = %s\n"
                "]",
            m_filename, m_scale.toString(), m_shift.toString(), m_linear_rgb);
    }

    Color3f eval(const Point2f &uv) {
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

        return m_linear_rgb ? Color3f{r, g, b}.toLinearRGB() : Color3f{r, g, b};
    }

protected:
    std::string m_filename;
    unsigned char *m_image;
    int m_height;
    int m_width;
    int m_channels;
    Vector2f m_scale;
    Vector2f m_shift;
    bool m_linear_rgb;
};

NORI_REGISTER_CLASS(ImageTexture, "image_texture")
NORI_NAMESPACE_END
