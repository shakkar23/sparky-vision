// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019-24 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // RealSense Cross Platform API
#include <iostream>
#include <SDL2/SDL.h>

#define OLC_PGE_APPLICATION
#include "olcPGESDL.h"

#include <ranges>
#include <vector>
#include <algorithm>
#include <bitset>
#include <execution>
#include <memory>

struct Color {
    uint8_t b;
    uint8_t g;
    uint8_t r;
    uint8_t a;
};


static Color hsv_to_rgb_manual(float h_degrees) {
    Color result = { 0, 0, 0, 0 };
    float r_float, g_float, b_float;

    // We fix S=1.0 and V=1.0 for maximum color saturation and brightness.
    // This simplifies the standard HSV-to-RGB formulas:
    // Chroma (C) = V * S = 1.0
    // M = V - C = 0.0

    // Ensure H is in the range [0.0, 360.0)
    h_degrees = fmod(h_degrees, 360.0f);
    if (h_degrees < 0.0f) {
        h_degrees += 360.0f;
    }

    // H' is the Hue scaled to sextants (0 to 6)
    float h_prime = h_degrees / 60.0f;

    // X is a value used in the conversion
    // X = C * (1 - |fmod(H', 2) - 1|)
    // Since C = 1.0f, X = 1.0f * (1 - |fmod(H', 2) - 1|)
    float x = 1.0f - fabs(fmod(h_prime, 2.0f) - 1.0f);

    // Determine the primary RGB triplet (R', G', B') based on the sextant
    if (h_prime >= 0.0f && h_prime < 1.0f) {        // resources.Red . Yellow
        r_float = 1.0f; g_float = x; b_float = 0.0f;
    }
    else if (h_prime >= 1.0f && h_prime < 2.0f) { // resources.Yellow . Green
        r_float = x; g_float = 1.0f; b_float = 0.0f;
    }
    else if (h_prime >= 2.0f && h_prime < 3.0f) { // resources.Green . Cyan
        r_float = 0.0f; g_float = 1.0f; b_float = x;
    }
    else if (h_prime >= 3.0f && h_prime < 4.0f) { // resources.Cyan . Blue
        r_float = 0.0f; g_float = x; b_float = 1.0f;
    }
    else if (h_prime >= 4.0f && h_prime < 5.0f) { // resources.Blue . Magenta
        r_float = x; g_float = 0.0f; b_float = 1.0f;
    }
    else { // (h_prime >= 5.0f && h_prime < 6.0f) // resources.Magenta . Red
        r_float = 1.0f; g_float = 0.0f; b_float = x;
    }

    // Final RGB is R = (R' + M) * 255. Since M = 0.0, it's R' * 255
    result.r = (uint8_t)(r_float * 255.0);
    result.g = (uint8_t)(g_float * 255.0);
    result.b = (uint8_t)(b_float * 255.0);

    return result;
}
static Color float_to_roygbiv(float x) {
    // 1. Clamp the input float to the [0.0, 1.0] range
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    // 2. Map the float to a Hue angle in degrees (0.0 to 300.0)
    // Red (0resources.) . Violet (300)
    float h_degrees = x * 300.0f;

    // 3. Convert the Hue to RGB using the manual formula
    return hsv_to_rgb_manual(h_degrees);
}

class GUI : public olc::PixelGameEngine
{
public:
    GUI() {
        resources.p = std::make_unique<rs2::pipeline>();
        resources.p->start();
        depth_sprite.width = 0;
        depth_sprite.height = 0;
    }

    struct realsense_resources {
        void reset() {
            depth_frame = rs2::depth_frame::frame();
            frames = rs2::frameset();
            p.reset(nullptr);
            p = std::make_unique<rs2::pipeline>();
            p->start();
        }
        std::unique_ptr<rs2::pipeline> p;
        rs2::depth_frame depth_frame = rs2::depth_frame::frame();
        rs2::frameset frames;
    } resources;

    std::vector<float> distances;
    olc::Sprite depth_sprite;
	int picked_x = 0;
	int picked_y = 0;
	float scaling_factor = 1.0f;
    bool OnUserCreate() override {
        this->sAppName = "RealSense Depth Viewer";
        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override {
        try {
            // Block program until frames arrive
            resources.frames = resources.p->wait_for_frames(1000u);
        }
        catch (std::exception& err) {
            bool initialized = false;
            for (int i = 0; i < 2; i++) {
                try {
                    resources.reset();
                    resources.frames = resources.p->wait_for_frames(1000u);
                    initialized = true;
                    break;
                }
                catch (std::exception& inner_err) {
                    continue;
                }
            }
            if (!initialized) {
                std::cerr << "Error during wait_for_frames: " << err.what() << std::endl;
                return false;
            }
        }

        // Try to get a frame of a depth image
        resources.depth_frame = resources.frames.get_depth_frame();

        // make the buffer larger if needed
        if (distances.capacity() < resources.depth_frame.get_width() * resources.depth_frame.get_height())
            distances.reserve(resources.depth_frame.get_width() * resources.depth_frame.get_height());

        auto bytes_per_px = resources.depth_frame.get_bytes_per_pixel();
        auto ptr = (const uint8_t*)resources.depth_frame.get_data();
        auto stride_in_bytes = resources.depth_frame.get_stride_in_bytes();

        if (
            depth_sprite.width != resources.depth_frame.get_width() ||
            depth_sprite.height != resources.depth_frame.get_height()
            ) {
            olc::Sprite new_sprite(resources.depth_frame.get_width(), resources.depth_frame.get_height());
            std::swap(depth_sprite, new_sprite);
        }

        auto heights = std::views::iota(0, resources.depth_frame.get_height());
        std::for_each(std::execution::unseq, heights.begin(), heights.end(), [&](auto y) {
            for (int x = 0; x < resources.depth_frame.get_width(); x++) {
                if (bytes_per_px == 2) {
                    uint16_t data = *((uint16_t*)(ptr + (y * stride_in_bytes) + (x * bytes_per_px)));
                    float normalized_depth = float(data) / float(std::numeric_limits<uint16_t>::max());
                    auto color = float_to_roygbiv(normalized_depth * 5.f);
                    depth_sprite.SetPixel(x, y, olc::Pixel(color.r, color.g, color.b));
                }
                else if (bytes_per_px == 4) {
                    uint32_t data = *((uint32_t*)(ptr + (y * stride_in_bytes) + (x * bytes_per_px)));
                    uint8_t gray = data / (256 * 256);
                    gray = 255 - gray;
                    depth_sprite.SetPixel(x, y, olc::Pixel(gray, gray, gray));
                }
            }
        });

		// Handle mouse picking
		picked_x = GetMouseX();
		picked_y = GetMouseY();
		if (picked_x >= 0 && picked_x < resources.depth_frame.get_width() &&
			picked_y >= 0 && picked_y < resources.depth_frame.get_height()) {
			float distance = resources.depth_frame.get_distance(picked_x, picked_y) * scaling_factor;
            // std::cout << "Picked pixel (" << picked_x << ", " << picked_y << ") - Distance: " << distance << " meters" << std::endl;

            // clear console first
			std::cout << "\033[2J\033[1;1H"; // ANSI escape codes to clear screen and move cursor to top-left
            
			// be sure to go back one line in the console so that we don't flood it with lines
			std::cout << "x: " << picked_x <<" y: " << picked_y << " = (" << picked_x << ", " << picked_y << ") - Distance: " << distance << " meters" << std::endl;
			std::cout << "scaling factor: " << scaling_factor << std::endl;
		}

        if(GetKey(olc::Key::UP).bPressed)
            scaling_factor *= 1.06f;
        if (GetKey(olc::Key::DOWN).bPressed)
			scaling_factor /= 1.06f;


        /*
        for (int y = 0; y < depth_sprite.height; y++) {
            for (int x = 0; x < depth_sprite.width; x++) {
                ;// auto d = distances[y * width + x];
            }
        }
        */

        // Draw the depth image
        DrawSprite(0, 0, &depth_sprite);
        return true;
    }

    void normalize_hitmap_awyzza(std::vector<float>& distances) {
        auto copy = distances;
        std::array<float, 256> thresholds{};
        for (int i = 0; i < 256; i++) {
            std::nth_element(copy.begin(), copy.begin() + (copy.size() * i) / 256, copy.end());
            thresholds[i] = copy[(copy.size() * i) / 256];
        }

        for (auto& d : distances) {
            d = std::distance(thresholds.begin(), std::ranges::lower_bound(thresholds, d));
            d = std::clamp(d, 0.0f, 255.0f);
        }
    }
};

int main(int argc, char* argv[]) try
{
    GUI gui;
    if (gui.Construct(848, 480, 1, 1))
        gui.Start();

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (...) {
    return EXIT_FAILURE;
}