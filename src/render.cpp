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

#include <nori/render.h>
#include <nori/parser.h>
#include <nori/scene.h>
#include <nori/camera.h>
#include <nori/block.h>
#include <nori/timer.h>
#include <nori/bitmap.h>
#include <nori/sampler.h>
#include <nori/integrator.h>
#include <nori/gui.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <filesystem/resolver.h>
#include <tbb/concurrent_vector.h>


NORI_NAMESPACE_BEGIN

RenderThread::RenderThread(ImageBlock & block) :
        m_block(block)
{
    m_render_status = 0;
    m_progress = 1.f;
}
RenderThread::~RenderThread() {
    stopRendering();
}

bool RenderThread::isBusy() {
    if(m_render_status == 3) {
        m_render_thread.join();
        m_render_status = 0;
    }
    return m_render_status != 0;
}

void RenderThread::stopRendering() {
    if(isBusy()) {
        cout << "Requesting interruption of the current rendering" << endl;
        m_render_status = 2;
        m_render_thread.join();
        m_render_status = 0;
        cout << "Rendering successfully aborted" << endl;
    }
}

float RenderThread::getProgress() {
    if(isBusy()) {
        return m_progress;
    }
    else return 1.f;
}

static void renderBlock(const Scene *scene, Sampler *sampler, ImageBlock &block, float contribution) {
    const Camera *camera = scene->getCamera();
    const Integrator *integrator = scene->getIntegrator();

    Point2i offset = block.getOffset();
    Vector2i size  = block.getSize();

    /* Clear the block contents */
    block.clear();

    /* For each pixel and pixel sample sample */
    for (int y=0; y<size.y(); ++y) {
        for (int x=0; x<size.x(); ++x) {
            Point2f pixelSample = Point2f((float) (x + offset.x()), (float) (y + offset.y())) + sampler->next2D();
            Point2f apertureSample = sampler->next2D();

            /* Sample a ray from the camera */
            Ray3f ray;
            Color3f value = camera->sampleRay(ray, pixelSample, apertureSample, contribution);

            /* Compute the incident radiance */
            value *= integrator->Li(scene, sampler, ray);

            /* Store in the image block */
            block.put(pixelSample, value);
        }
    }
}

void RenderThread::renderScene(const std::string & filename) {
    // TODO (gimoro): Make this a CLI argument or allow option in scene xml file
    bool computeVariance = true;

    filesystem::path path(filename);

    /* Add the parent directory of the scene file to the
       file resolver. That way, the XML file can reference
       resources (OBJ files, textures) using relative paths */
    getFileResolver()->prepend(path.parent_path());

    NoriObject* root = loadFromXML(filename);

    // When the XML root object is a scene, start rendering it ..
    if (root->getClassType() == NoriObject::EScene) {
        m_scene = static_cast<Scene *>(root);

        const Camera *camera_ = m_scene->getCamera();
        m_scene->getIntegrator()->preprocess(m_scene);

        /* Allocate memory for the entire output image and clear it */
        m_block.init(camera_->getOutputSize(), camera_->getReconstructionFilter());
        m_block.clear();

        /* Determine the filename of the output bitmap */
        std::string outputName = filename;
        size_t lastdot = outputName.find_last_of(".");
        if (lastdot != std::string::npos)
            outputName.erase(lastdot, std::string::npos);

        std::string varianceOutputName = outputName + "_variance.exr";
        outputName += ".exr";

        /* Do the following in parallel and asynchronously */
        m_render_status = 1;
        m_render_thread = std::thread([this, outputName, computeVariance, varianceOutputName] {
            const Camera *camera = m_scene->getCamera();
            Vector2i outputSize = camera->getOutputSize();

            /* Create a block generator (i.e. a work scheduler) */
            BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);

            cout << "Rendering .. ";
            cout.flush();
            Timer timer;

            auto numSamples = m_scene->getSampler()->getSampleCount();
            auto numBlocks = blockGenerator.getBlockCount();

            tbb::concurrent_vector< std::unique_ptr<Sampler> > samplers;
            samplers.resize(numBlocks);

            // Variance estimation using "Bessel's correction"
            ImageBlock varianceBlock{camera->getOutputSize(), camera->getReconstructionFilter()};
            Bitmap sumBitmap{camera->getOutputSize()};
            Bitmap sumSquaredBitmap{camera->getOutputSize()};

            for (uint32_t k = 0; k < numSamples ; ++k) {
                m_progress = k/float(numSamples);
                if(m_render_status == 2)
                    break;

                tbb::blocked_range<int> range(0, numBlocks);

                if (computeVariance) varianceBlock.clear();

                auto map = [&](const tbb::blocked_range<int> &range) {
                    // Allocate memory for a small image block to be rendered by the current thread
                    ImageBlock block(Vector2i(NORI_BLOCK_SIZE),
                                     camera->getReconstructionFilter());

                    for (int i = range.begin(); i < range.end(); ++i) {
                        // Request an image block from the block generator
                        blockGenerator.next(block);

                        // Get block id to continue using the same sampler
                        auto blockId = block.getBlockId();
                        if(k == 0) { // Initialize the sampler for the first sample
                            std::unique_ptr<Sampler> sampler(m_scene->getSampler()->clone());
                            sampler->prepare(block);
                            samplers.at(blockId) = std::move(sampler);
                        }

                        // Render all contained pixels
                        renderBlock(m_scene, samplers.at(blockId).get(), block,m_progress);
                        // The image block has been processed. Now add it to the "big" block that represents the entire image
                        m_block.put(block);        

                        if (computeVariance) varianceBlock.put(block);
                    }
                };

                /// Uncomment the following line for single threaded rendering
                //map(range);

                /// Default: parallel rendering
                tbb::parallel_for(range, map);

                if (computeVariance) {
                    // Variance estimation (update sum and sumSquared)
                    varianceBlock.lock();
                    Bitmap bitmap{*varianceBlock.toBitmap()};
                    varianceBlock.unlock();

                    auto rows = bitmap.rows();
                    auto columns = bitmap.cols();
                    Color3f currentPixel;
                    for (int y = 0; y < rows; y++) {    
                        for (int x = 0; x < columns; x++) {    
                            currentPixel = bitmap(y, x);
                            sumBitmap(y, x) += currentPixel;
                            sumSquaredBitmap(y, x) += pow(currentPixel, 2);
                        }
                    }
                }
                

                blockGenerator.reset();
            }

            cout << "done. (took " << timer.elapsedString() << ")" << endl;

            /* Now turn the rendered image block into
               a properly normalized bitmap */
            m_block.lock();
            std::unique_ptr<Bitmap> bitmap(m_block.toBitmap());
            m_block.unlock();

            /* Save using the OpenEXR format */
            bitmap->save(outputName);

            if (computeVariance) {
                // Variance estimation
                Bitmap varianceBitmap{camera->getOutputSize()};
                auto rows = varianceBitmap.rows();
                auto columns = varianceBitmap.cols();
                float samplesFactor = numSamples * (numSamples - 1);
                for (int y = 0; y < rows; y++) {
                    for (int x = 0; x < columns; x++) {
                        // var[p] = (sample variance) * 1/n = ((ss/n - (s/n)^2) * (n/(n-1))) * 1/n = (ss - s^2/n) / ((n-1) * n)
                        varianceBitmap(y, x) = (sumSquaredBitmap(y, x) - (pow(sumBitmap(y, x), 2) / numSamples)) / samplesFactor;
                    }
                }
                
                // Save variance bitmap
                varianceBitmap.save(varianceOutputName);
            }
            

            delete m_scene;
            m_scene = nullptr;

            m_render_status = 3;
        });

    }
    else {
        delete root;
    }

}


NORI_NAMESPACE_END