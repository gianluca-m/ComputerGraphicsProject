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
        m_filename = props.getString("filename");
        m_shift = props.getVector2("shift", Vector2f(0.0f));

        auto filepath = getFileResolver()->resolve(m_filename);

        m_image = stbi_load(filepath.str().c_str(), &m_width, &m_height, &m_channels, 0);

        if (m_image == NULL) {
            tfm::printfln("Loading image texture '%s' failed", m_filename);
        }
        else {
            tfm::printfln("Loading image texture successful. #channels = %i, width = %i, height = %i", m_channels, m_width, m_height);
        }
    }

    std::string toString() const {
        return tfm::format(
            "ImageTexture[\n"
                "  filename = %s\n"
                "]",
            m_filename);
    }

    Color3f eval(const Point2f &uv) {
        auto x = (int) std::round(uv.x() * m_width);
        auto y = (int) std::round((1.0 - uv.y()) * m_height);

        if (x < 0) x = 0; 
        if (y < 0) y = 0; 

        if (x > m_width - 1) x = m_width - 1;
        if (y > m_height - 1) y = m_height - 1;

        x = (int) (x + m_shift.x() * m_width) % m_width;        
        y = (int) (y + m_shift.y() * m_height) % m_height;

        int index = (x + y * m_width) * m_channels;
        auto r = (float) (m_image[index] / 255.0);
        auto g = (float) (m_image[index + 1] / 255.0);
        auto b = (float) (m_image[index + 2] / 255.0);

        return Color3f{r, g, b};
    }

protected:
    std::string m_filename;
    unsigned char *m_image;
    int m_height;
    int m_width;
    int m_channels;
    Vector2f m_shift;
};

NORI_REGISTER_CLASS(ImageTexture, "image_texture")
NORI_NAMESPACE_END
