#ifndef CAMERA_H
#define CAMERA_H
//==============================================================================================
// Originally written in 2016 by Peter Shirley <ptrshrl@gmail.com>
//
// To the extent possible under law, the author(s) have dedicated all copyright and related and
// neighboring rights to this software to the public domain worldwide. This software is
// distributed without any warranty.
//
// You should have received a copy (see file COPYING.txt) of the CC0 Public Domain Dedication
// along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//==============================================================================================

#include "rtweekend.h"

#include "color.h"
#include "hittable.h"
#include "material.h"

#include <functional>
#include <future>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

template <typename T>
class BlockingQueue {
  private:
    std::mutex _mtx;
    std::condition_variable _cond;
    int _max_size;
    std::queue<T> _queue;

  public:
    BlockingQueue(int max_size) : _max_size(max_size) {
    }

    void push(T t) {
        std::unique_lock<std::mutex> lock(_mtx);
        _cond.wait(lock, [this]() { return _queue.size() < _max_size; });
        _queue.push(t);
        lock.unlock();
        _cond.notify_one();
    }

    T front() {
        std::unique_lock<std::mutex> lock(_mtx);
        _cond.wait(lock, [this]() { return !_queue.empty(); });
        return _queue.front();
    }

    void pop() {
        std::unique_lock<std::mutex> lock(_mtx);
        _cond.wait(lock, [this]() { return !_queue.empty(); });
        _queue.pop();
        lock.unlock();
        _cond.notify_one();
    }

    int size() {
        std::lock_guard<std::mutex> lock(_mtx);
        return _queue.size();
    }
};

void producer_func(BlockingQueue<std::shared_future<std::vector<color>>> &futures, int image_height, int image_width, std::function<std::vector<color>(int, int)> lambda_func) {
    for (int j = 0; j < image_height; ++j) {
        // A new thread is spawned and pushed into BlockingQueue.
        // If BlockingQueue is full, this thread will unlock the lock and wait until an element is pop out.
        std::shared_future<std::vector<color>> f = std::async(std::launch::async, lambda_func, j, image_width);
        futures.push(f);
    }
}

class camera {
  public:
    double aspect_ratio      = 1.0;    // Ratio of image width over height
    int    image_width       = 100;    // Rendered image width in pixel count
    int    samples_per_pixel = 10;     // Count of random samples for each pixel
    int    max_depth         = 10;     // Maximum number of ray bounces into scene
    color  background;                 // Scene background color

    double vfov     = 90;              // Vertical view angle (field of view)
    point3 lookfrom = point3(0,0,-1);  // Point camera is looking from
    point3 lookat   = point3(0,0,0);   // Point camera is looking at
    vec3   vup      = vec3(0,1,0);     // Camera-relative "up" direction

    double defocus_angle = 0;  // Variation angle of rays through each pixel
    double focus_dist = 10;    // Distance from camera lookfrom point to plane of perfect focus

    void render(const hittable& world) {
        initialize();

        std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

        const int CONCURRENCY = 4; // `std::thread::hardware_concurrency();` returns 16 on my laptop.
        std::clog << "\rCONCURRENCY: " << CONCURRENCY << std::endl;
        BlockingQueue<std::shared_future<std::vector<color>>> futures(CONCURRENCY); // CONCURRENCY = BlockingQueue's size

        // The lambda function on which every spawned thread is going to run. Every
        // thread process the whole given j-th row.
        auto lambda_func = [this, &world](int j, int width) {
            std::vector<color> vec_pixel_color(width, color(0, 0, 0));
            for (int i = 0; i < width; ++i) {
                color pixel_color(0,0,0);
                for (int sample = 0; sample < this->samples_per_pixel; ++sample) {
                    const ray r = get_ray(i, j);
                    pixel_color += ray_color(r, this->max_depth, world);
                }
                vec_pixel_color[i] = pixel_color;
            }
            return vec_pixel_color;
        };
        std::thread producer_thread(producer_func, std::ref(futures), image_height, image_width, lambda_func);

        for (int j = 0; j < image_height; ++j) {
            std::clog << "\rScanlines remaining: " << (image_height - j) << ' ' << std::flush;

            // Multi thread
            std::shared_future<std::vector<color>> f = futures.front();
            std::vector<color> vec_pixel_color = f.get();
            futures.pop();
            for (int i = 0; i < vec_pixel_color.size(); ++i) {
                write_color(std::cout, vec_pixel_color[i], samples_per_pixel);
            }

            // Single thread
            // for (int i = 0; i < image_width; ++i) {
            //     color pixel_color(0,0,0);
            //     for (int sample = 0; sample < samples_per_pixel; ++sample) {
            //         ray r = get_ray(i, j);
            //         pixel_color += ray_color(r, max_depth, world);
            //     }
            //     write_color(std::cout, pixel_color, samples_per_pixel);
            // }
        }
        producer_thread.join();

        std::clog << "\rDone.                 \n";
    }

  private:
    int    image_height;    // Rendered image height
    point3 center;          // Camera center
    point3 pixel00_loc;     // Location of pixel 0, 0
    vec3   pixel_delta_u;   // Offset to pixel to the right
    vec3   pixel_delta_v;   // Offset to pixel below
    vec3   u, v, w;         // Camera frame basis vectors
    vec3   defocus_disk_u;  // Defocus disk horizontal radius
    vec3   defocus_disk_v;  // Defocus disk vertical radius

    void initialize() {
        image_height = static_cast<int>(image_width / aspect_ratio);
        image_height = (image_height < 1) ? 1 : image_height;

        center = lookfrom;

        // Determine viewport dimensions.
        auto theta = degrees_to_radians(vfov);
        auto h = tan(theta/2);
        auto viewport_height = 2 * h * focus_dist;
        auto viewport_width = viewport_height * (static_cast<double>(image_width)/image_height);

        // Calculate the u,v,w unit basis vectors for the camera coordinate frame.
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);

        // Calculate the vectors across the horizontal and down the vertical viewport edges.
        vec3 viewport_u = viewport_width * u;    // Vector across viewport horizontal edge
        vec3 viewport_v = viewport_height * -v;  // Vector down viewport vertical edge

        // Calculate the horizontal and vertical delta vectors to the next pixel.
        pixel_delta_u = viewport_u / image_width;
        pixel_delta_v = viewport_v / image_height;

        // Calculate the location of the upper left pixel.
        auto viewport_upper_left = center - (focus_dist * w) - viewport_u/2 - viewport_v/2;
        pixel00_loc = viewport_upper_left + 0.5 * (pixel_delta_u + pixel_delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;
    }

    ray get_ray(int i, int j) const {
        // Get a randomly-sampled camera ray for the pixel at location i,j, originating from
        // the camera defocus disk.

        auto pixel_center = pixel00_loc + (i * pixel_delta_u) + (j * pixel_delta_v);
        auto pixel_sample = pixel_center + pixel_sample_square();

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;
        auto ray_time = random_double();

        return ray(ray_origin, ray_direction, ray_time);
    }

    vec3 pixel_sample_square() const {
        // Returns a random point in the square surrounding a pixel at the origin.
        auto px = -0.5 + random_double();
        auto py = -0.5 + random_double();
        return (px * pixel_delta_u) + (py * pixel_delta_v);
    }

    vec3 pixel_sample_disk(double radius) const {
        // Generate a sample from the disk of given radius around a pixel at the origin.
        auto p = radius * random_in_unit_disk();
        return (p[0] * pixel_delta_u) + (p[1] * pixel_delta_v);
    }

    point3 defocus_disk_sample() const {
        // Returns a random point in the camera defocus disk.
        auto p = random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, int depth, const hittable& world) const {
        // If we've exceeded the ray bounce limit, no more light is gathered.
        if (depth <= 0)
            return color(0,0,0);

        hit_record rec;

        // If the ray hits nothing, return the background color.
        if (!world.hit(r, interval(0.001, infinity), rec))
            return background;

        ray scattered;
        color attenuation;
        color color_from_emission = rec.mat->emitted(rec.u, rec.v, rec.p);

        if (!rec.mat->scatter(r, rec, attenuation, scattered))
            return color_from_emission;

        color color_from_scatter = attenuation * ray_color(scattered, depth-1, world);

        return color_from_emission + color_from_scatter;
    }
};


#endif
