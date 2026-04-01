#include <ufomap_bundled/ufomap_mapping/ufomapGPU.cuh>

// Macro source: https://stackoverflow.com/questions/14038589/what-is-the-canonical-way-to-check-for-errors-using-the-cuda-runtime-api
#define gpuErrchk(ans)                        \
    {                                         \
        gpuAssert((ans), __FILE__, __LINE__); \
    }

inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort = true)
{

    if (code != cudaSuccess)
    {
        fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
        if (abort)
            exit(code);
    }
}

__global__ void infoGainKernel(double *out, ufo::map::OccupancyMapColor *map, ufo::map::Point3 *origin, ufo::map::Point3 *endpoints,
                               double *max_range, int *num_endpoints, int *num_startpoints)
{
    // Get the index of the current thread
    int index_x = threadIdx.x + blockIdx.x * blockDim.x;
    int index_y = threadIdx.y + blockIdx.y * blockDim.y;
    if ((index_x < num_endpoints[0]) && (index_y < num_startpoints[0])) // if index outside of range, do nothing to avoid memory access errors
    {
        int index = index_x + index_y * num_endpoints[0];

        // Check if the endpoint and startpoint is within the map
        double min_allowed = map->getMin()[0];
        double max_allowed = map->getMax()[0];

        assert((min_allowed <= origin[blockIdx.y].min() && max_allowed >= origin[blockIdx.y].max() && min_allowed <= endpoints[index].min() &&
                max_allowed >= endpoints[index].max()));

        // Calculate the direction and distance of the ray
        ufo::map::Point3 direction = (endpoints[index] - origin[blockIdx.y]);
        double distance = direction.norm();
        direction /= distance;

        // Check if the ray is within the max range
        if (0 <= max_range[0] && distance > max_range[0])
        {
            endpoints[index] = origin[blockIdx.y] + (direction * max_range[0]);
            distance = max_range[0];
        }

        // Define variables for the raycasting
        ufo::map::Key current;
        ufo::map::Key ending;
        std::array<int, 3> step;
        ufo::map::Point3 t_delta;
        ufo::map::Point3 t_max;
        double info_val = 0;

        // Compute the raycasting
        map->computeRayInit(origin[blockIdx.y], endpoints[index], direction, current, ending, step, t_delta, t_max);
        // Increment
        int i = 0;
        bool occupied = false;
        while ((current.getDepth() != ending.getDepth() || !current.equals(ending)) && t_max.min() <= distance)
        {
            i++;
            if (!occupied)
            {
                // Compute the information gain for the current node
                ufo::map::OccupancyState state = map->getState(current);

                switch (state)
                {
                case ufo::map::OccupancyState::occupied: // If the node is occupied, we
                                                         // want to know less about it
                                                         // --> Onyl small increae
                    info_val = info_val + (1 - map->getOccupancy(current));
                    occupied = true;
                    break;
                case ufo::map::OccupancyState::free: // If the node is free,  we
                                                     // want to know less about it
                                                     // --> Onyl small increae
                    info_val = info_val + (map->getOccupancy(current));

                    break;
                case ufo::map::OccupancyState::unknown: // If the node is unknown -->
                                                        // We want to know more
                    info_val++;
                    break;
                }
            }
            // Increment the ray
            map->computeRayTakeStep(current, step, t_delta, t_max);
        }
        // Write the information gain to the output array
        out[index] = info_val / i;
    }
}

__global__ void occlusionGainKernel(double *out, ufo::map::OccupancyMapColor *map, ufo::map::Point3 *origin, ufo::map::Point3 *endpoints,
                                    double *max_range, int *num_endpoints, int *num_startpoints, int *num_pred_occ)
{
    // Get the index of the current thread
    // printf("in occlusion gain kernel \n");
    int index_x = threadIdx.x + blockIdx.x * blockDim.x;
    int index_y = threadIdx.y + blockIdx.y * blockDim.y;
    int index_z = threadIdx.z + blockIdx.z * blockDim.z;

    // printf("num_pred_occ= %d \n", num_pred_occ[0]);
    if ((index_x < num_endpoints[0]) && (index_y < num_startpoints[0]) && (index_z < num_pred_occ[0])) // if index outside of range, do nothing to avoid memory access errors
    {
        int index = index_x * num_startpoints[0] + index_z * num_endpoints[0]; // index equals to thread + prediction step * number of endpoints

        // Check if the endpoint and startpoint is within the map
        double min_allowed = map->getMin()[0];
        double max_allowed = map->getMax()[0];

        // check if endpint and startpoints are numbers
        // if (isnan(origin[blockIdx.y].min()) || isnan(origin[blockIdx.y].max()) || isnan(endpoints[index].min()) || isnan(endpoints[index].max()))
        // {
        //     printf("origin_min= %f, origin_max= %f, endpoint_min= %f, endpoint_max= %f \n", origin[blockIdx.y].min(), origin[blockIdx.y].max(), endpoints[index].min(), endpoints[index].max());
        //     out[index] = 0;
        //     return;
        // }

        assert((min_allowed <= origin[blockIdx.y].min() && max_allowed >= origin[blockIdx.y].max() && min_allowed <= endpoints[index_x].min() &&
                max_allowed >= endpoints[index_x].max()));

        // Calculate the direction and distance of the ray
        ufo::map::Point3 direction = (endpoints[index_x] - origin[blockIdx.y]);
        double distance = direction.norm();
        direction /= distance;

        // Check if the ray is within the max range
        if (0 <= max_range[0] && distance > max_range[0])
        {
            endpoints[index_x] = origin[blockIdx.y] + (direction * max_range[0]);
            distance = max_range[0];
        }

        // Define variables for the raycasting
        ufo::map::Key current;
        ufo::map::Key ending;
        std::array<int, 3> step;
        ufo::map::Point3 t_delta;
        ufo::map::Point3 t_max;
        double info_val = 0;

        // Compute the raycasting
        map->computeRayInit(origin[blockIdx.y], endpoints[index_x], direction, current, ending, step, t_delta, t_max);
        // Increment
        int i = 0;
        bool occupied = false;
        while ((current.getDepth() != ending.getDepth() || !current.equals(ending)) && t_max.min() <= distance && !occupied)
        {
            i++;
            // Compute the information gain for the current node
            ufo::map::OccupancyState state = map->getState(current);
            ufo::map::Prediction pred = map->getPrediction(current);
            ufo::map::Color color = map->getColor(current);

            if (index_z == 0 && color.r != 255) // Current time
            {

                switch (state)
                {
                case ufo::map::OccupancyState::occupied: // if the node is occupied it is occluded
                    info_val = 0;
                    occupied = true;
                    break;
                case ufo::map::OccupancyState::free: // If the node is free,  the hands are not occluded
                    info_val++;
                    break;
                case ufo::map::OccupancyState::unknown: // If the node is unknown the hands are maybe occluded
                                                        // TODO(renz): Maybe add a smaller increase here instead of 1
                    info_val++;
                    break;
                }
            }
            else if (index_z == 0 && color.r == 255) // Current time
            {
                info_val++;
            }
            else if (index_z < num_pred_occ[0])
            {
                // ONLY CONSIDER MANUALLY ADDED OBSTACLES INCLUDING PREDICTIONS
                if (pred.getPredictionAtElement(index_z))
                {
                    info_val = 0;
                    occupied = true;
                }
                else
                {
                    info_val++;
                }
            }
            else
            {
                printf("UfomapGPU: index_z out of range \n");
                // printf("index_z= %d, num_pred_occ= %d \n", index_z, num_pred_occ[0]);
            }
            // Increment the ray
            map->computeRayTakeStep(current, step, t_delta, t_max);
        }

        out[index] = info_val / i;
        return;
    }
}

namespace ufomapGPU
{
    void UFOMapGPU::copyMapCPUtoGPU(ufo::map::OccupancyMapColor *map_host)
    {
        // Initialize host map pointer (only once since it doesn't change)
        _h_map = map_host;

        // Allocate memory on the device
        gpuErrchk(cudaMalloc((void **)&_d_map, sizeof(ufo::map::OccupancyMapColor))); // alloc map space

        // Update the map on the GPU
        // updateMapGPU();
    };

    void UFOMapGPU::updateMapGPU()
    {
        // copy current host map; TODO(renz): Check if this is necessary or if it's possible to access directly the host map (up to now I always get a
        // segfault when trying to use the host map directly)
        ufo::map::OccupancyMapColor host_map(*_h_map);

        // Free pointer map
        _h_children_map.clear();

        // Get the children pointer for all nodes
        _depth_max = host_map.getTreeDepthLevels();
        INNER_NODE *root = &host_map.getRoot();
        getChildrenPointer(root, &host_map, 0, _depth_max);

        // Copy data to device (Note that the node pointers for children are already copied to the device)
        gpuErrchk(cudaMemcpy(_d_map, &host_map, sizeof(ufo::map::OccupancyMapColor), cudaMemcpyHostToDevice));

        // Required since the destructor needs the correct host pointers to free the memory on the host
        setPointerBackToHost(root, &host_map, 0, _depth_max);

        // Free the copied host map
        host_map.clear();
    };

    void UFOMapGPU::clearMapGPU()
    { // Free memory on device for map
        cudaFree(_d_map);
    };

    void UFOMapGPU::calcInfoGain(int num, int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points,
                                 std::map<int, std::vector<ufo::map::Point3>> *endpoints, double max_range, Eigen::MatrixXd *results,
                                 Eigen::VectorXd *time_metrics)
    {
        std::chrono::time_point t1 = std::chrono::steady_clock::now();

        // Initialize variables
        if (_h_map == nullptr)
        {
            printf("Map not copied\n");
            return;
        }

        // Update the map on the GPU (including children pointers)
        updateMapGPU();

        // Map timestamp t2
        std::chrono::time_point t2_info = std::chrono::steady_clock::now();

        // Information distribution preparation

        // host copy of output
        double *out;

        // Convert the input to the correct format
        std::vector<ufo::map::Point3> endpoints_h(start_points->size() * num);
        std::vector<ufo::map::Point3> startpoints_h(start_points->size());
        for (int i = 0; i < start_points->size(); i++)
        {
            startpoints_h[i] = std::get<1>(start_points->at(i));
            std::vector<ufo::map::Point3> *ep = &endpoints->at(std::get<0>(start_points->at(i)));
            for (int j = 0; j < num; j++)
            {
                endpoints_h[i * num + j] = ep->at(j);
            }
        }
        int num_startpoints = startpoints_h.size();

        // device copies of inputs and output
        double *d_out;
        int *d_num_endpoints, *d_num_startpoints;
        double *d_max_range;
        ufo::map::Point3 *d_endpoints;
        ufo::map::Point3 *d_start;

        // Alloc space for device copies
        // one double for each endpoint
        gpuErrchk(cudaMalloc((void **)&d_out, sizeof(double) * num * num_startpoints));
        // alloc space for endpoints
        gpuErrchk(cudaMalloc((void **)&d_endpoints, sizeof(ufo::map::Point3) * num * num_startpoints));
        // alloc space for startpoint (only one); TODO(renz): Check if parallelization for startpoints is also possible
        gpuErrchk(cudaMalloc((void **)&d_start, sizeof(ufo::map::Point3) * num_startpoints));
        gpuErrchk(cudaMalloc((void **)&d_max_range, sizeof(double)));    // alloc space for max_range double
        gpuErrchk(cudaMalloc((void **)&d_num_endpoints, sizeof(int)));   // alloc space for num_endpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_startpoints, sizeof(int))); // alloc space for num_startpoints int

        // Alloc space for host copies of info gain for all endpoints
        out = (double *)malloc(sizeof(double) * num * num_startpoints); // one double for each endpoint

        // device
        // copy max_range to device
        gpuErrchk(cudaMemcpy(d_max_range, &max_range, sizeof(double), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_endpoints, &num, sizeof(int), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_startpoints, &num_startpoints, sizeof(int), cudaMemcpyHostToDevice));
        // copy startpoint to device
        gpuErrchk(cudaMemcpy(d_start, startpoints_h.data(), sizeof(ufo::map::Point3) * num_startpoints, cudaMemcpyHostToDevice));
        // copy endpoints to device
        gpuErrchk(cudaMemcpy(d_endpoints, endpoints_h.data(), sizeof(ufo::map::Point3) * num * num_startpoints, cudaMemcpyHostToDevice));

        // Error check
        gpuErrchk(cudaPeekAtLastError());

        dim3 gridDim(static_cast<int>(std::ceil(static_cast<double>(num) / static_cast<double>(threads))), num_startpoints);

        std::chrono::time_point t3_info = std::chrono::steady_clock::now();

        std::chrono::time_point t4_info = std::chrono::steady_clock::now();
        // Launch kernel
        ::infoGainKernel<<<gridDim, threads>>>(d_out, _d_map, d_start, d_endpoints, d_max_range, d_num_endpoints, d_num_startpoints);

        // Synchronize threads
        // gpuErrchk(cudaDeviceSynchronize());
        std::chrono::time_point t5_info = std::chrono::steady_clock::now();

        std::chrono::time_point t6_info = std::chrono::steady_clock::now();

        // Copy output from device to host
        gpuErrchk(cudaMemcpy(out, d_out, sizeof(double) * num * num_startpoints, cudaMemcpyDeviceToHost));

        // Copy output to Eigen matrix
        for (int blockIdx = 0; blockIdx < num_startpoints; blockIdx++)
        {
            results->row(blockIdx) = Eigen::Map<Eigen::VectorXd>(out + blockIdx * num, num);
            // printf("GPU result= %f \n", results->row(blockIdx).mean());
        }

        // Free memory on host
        free(out);

        // Cleanup on device

        // Clean all the children pointers from all nodes (important to avoid memory leaks)
        for (auto it = _h_children_map.begin(); it != _h_children_map.end(); ++it)
        {
            cudaFree(it->first);
        }

        // Free all the device memory
        cudaFree(d_out);
        cudaFree(d_endpoints);
        cudaFree(d_start);
        cudaFree(d_max_range);
        cudaFree(d_num_endpoints);
        cudaFree(d_num_startpoints);

        // Timing
        std::chrono::time_point t7_info = std::chrono::steady_clock::now();
        std::chrono::time_point t_fin = std::chrono::steady_clock::now();
        // 1: map copy time; 2: copy info variables time 3: info kernel time; 4: cleanup time; 5: total data time (without kernel) 6: total time (incl. occlusion)
        *time_metrics << std::chrono::duration<float, std::chrono::seconds::period>(t2_info - t1).count(),
            std ::chrono::duration<float, std::chrono::seconds::period>(t3_info - t2_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t5_info - t4_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t7_info - t6_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_info - t2_info).count() + std::chrono::duration<float, std::chrono::seconds::period>(t7_info - t6_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t_fin - t1).count();
    };

    void UFOMapGPU::calcInfoAndOccGain(int num_info, int num_occ, int num_pred_occ, int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points_occ,
                                       std::map<int, std::vector<ufo::map::Point3>> *endpoints, std::map<int, std::vector<ufo::map::Point3>> *endpoints_occ, double max_range, Eigen::MatrixXd *results, Eigen::MatrixXd *results_occ,
                                       Eigen::VectorXd *time_metrics, Eigen::VectorXd *time_metrics_occ)
    {
        std::chrono::time_point t1 = std::chrono::steady_clock::now();

        // Initialize and update map
        if (_h_map == nullptr)
        {
            printf("Map not copied\n");
            return;
        }

        // Update the map on the GPU (including children pointers)
        updateMapGPU();

        // Map timestamp t2
        std::chrono::time_point t2_info = std::chrono::steady_clock::now();

        // Information distribution preparation
        // host copy of output
        double *out;

        // Convert the input to the correct format
        std::vector<ufo::map::Point3> endpoints_h(start_points->size() * num_info);
        std::vector<ufo::map::Point3> startpoints_h(start_points->size());

        for (int i = 0; i < start_points->size(); i++)
        {
            startpoints_h[i] = std::get<1>(start_points->at(i));
            std::vector<ufo::map::Point3> *ep = &endpoints->at(std::get<0>(start_points->at(i)));
            for (int j = 0; j < num_info; j++)
            {
                endpoints_h[i * num_info + j] = ep->at(j);
            }
        }

        int num_startpoints = startpoints_h.size();

        // device copies of inputs and output
        double *d_out;
        int *d_num_endpoints, *d_num_startpoints;
        double *d_max_range;
        ufo::map::Point3 *d_endpoints;
        ufo::map::Point3 *d_start;

        // Alloc space for device copies
        // one double for each endpoint
        gpuErrchk(cudaMalloc((void **)&d_out, sizeof(double) * num_info * num_startpoints));
        // alloc space for endpoints
        gpuErrchk(cudaMalloc((void **)&d_endpoints, sizeof(ufo::map::Point3) * num_info * num_startpoints));
        // alloc space for startpoint (only one); TODO(renz): Check if parallelization for startpoints is also possible
        gpuErrchk(cudaMalloc((void **)&d_start, sizeof(ufo::map::Point3) * num_startpoints));
        gpuErrchk(cudaMalloc((void **)&d_max_range, sizeof(double)));    // alloc space for max_range double
        gpuErrchk(cudaMalloc((void **)&d_num_endpoints, sizeof(int)));   // alloc space for num_endpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_startpoints, sizeof(int))); // alloc space for num_startpoints int

        // Alloc space for host copies of info gain for all endpoints
        out = (double *)malloc(sizeof(double) * num_info * num_startpoints); // one double for each endpoint

        // device
        // copy max_range to device
        gpuErrchk(cudaMemcpy(d_max_range, &max_range, sizeof(double), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_endpoints, &num_info, sizeof(int), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_startpoints, &num_startpoints, sizeof(int), cudaMemcpyHostToDevice));
        // copy startpoint to device
        gpuErrchk(cudaMemcpy(d_start, startpoints_h.data(), sizeof(ufo::map::Point3) * num_startpoints, cudaMemcpyHostToDevice));
        // copy endpoints to device
        gpuErrchk(cudaMemcpy(d_endpoints, endpoints_h.data(), sizeof(ufo::map::Point3) * num_info * num_startpoints, cudaMemcpyHostToDevice));

        dim3 gridDim(static_cast<int>(std::ceil(static_cast<double>(num_info) / static_cast<double>(threads))), num_startpoints);
        dim3 blockDim(threads, 1, 1);

        // Time stamps
        std::chrono::time_point t3_info = std::chrono::steady_clock::now();
        std::chrono::time_point t2_occ = std::chrono::steady_clock::now();

        // Occlusion distribution preparation
        bool skipOcc = false;
        if (num_occ == 0)
        {
            skipOcc = true;
        }

        // host copy of output
        double *out_occ;

        // Convert the input to the correct format
        std::vector<ufo::map::Point3> endpoints_h_occ(start_points_occ->size() * num_occ);
        std::vector<ufo::map::Point3> startpoints_h_occ(start_points_occ->size());

        if (start_points_occ->size() == endpoints_occ->size())
        {

            for (int i = 0; i < start_points_occ->size(); i++)
            {
                startpoints_h_occ[i] = std::get<1>(start_points_occ->at(i));
                // Out of range error here
                std::vector<ufo::map::Point3> *ep = &endpoints_occ->at(std::get<0>(start_points_occ->at(i)));
                for (int j = 0; j < num_occ; j++)
                {

                    endpoints_h_occ[i * num_occ + j] = ep->at(j);
                }
            }
        }
        else
        {
            printf("start_points_occ and endpoints_occ size mismatch \n");
            skipOcc = true;
        }
        int num_startpoints_occ = startpoints_h_occ.size();

        // device copies of inputs and output
        double *d_out_occ;
        int *d_num_endpoints_occ, *d_num_startpoints_occ, *d_num_pred_occ;
        double *d_max_range_occ;
        ufo::map::Point3 *d_endpoints_occ;
        ufo::map::Point3 *d_start_occ;

        // Alloc space for device copies
        // one double for each endpoint
        gpuErrchk(cudaMalloc((void **)&d_out_occ, sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ));
        gpuErrchk(cudaMalloc((void **)&d_endpoints_occ, sizeof(ufo::map::Point3) * num_occ * num_startpoints_occ));
        gpuErrchk(cudaMalloc((void **)&d_start_occ, sizeof(ufo::map::Point3) * num_startpoints_occ));
        gpuErrchk(cudaMalloc((void **)&d_max_range_occ, sizeof(double)));    // alloc space for max_range double
        gpuErrchk(cudaMalloc((void **)&d_num_endpoints_occ, sizeof(int)));   // alloc space for num_endpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_startpoints_occ, sizeof(int))); // alloc space for num_startpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_pred_occ, sizeof(int)));        // alloc space for num_pred int

        // Alloc space for host copies of info gain for all endpoints
        out_occ = (double *)malloc(sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ); // one double for each endpoint

        gpuErrchk(cudaMemcpy(d_max_range_occ, &max_range, sizeof(double), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_endpoints_occ, &num_occ, sizeof(int), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_startpoints_occ, &num_startpoints_occ, sizeof(int),
                             cudaMemcpyHostToDevice));
        // copy num_pred to device
        gpuErrchk(cudaMemcpy(d_num_pred_occ, &num_pred_occ, sizeof(int), cudaMemcpyHostToDevice));

        // copy startpoint to device
        gpuErrchk(cudaMemcpy(d_start_occ, startpoints_h_occ.data(), sizeof(ufo::map::Point3) * num_startpoints_occ, cudaMemcpyHostToDevice));
        // copy endpoints to device
        gpuErrchk(cudaMemcpy(d_endpoints_occ, endpoints_h_occ.data(), sizeof(ufo::map::Point3) * num_occ * num_startpoints_occ, cudaMemcpyHostToDevice));

        // std::cout << "Num occ: " << num_occ << std::endl;
        dim3 gridDim_occ(static_cast<int>(std::ceil(static_cast<double>(num_occ) / static_cast<double>(threads))), num_startpoints_occ, num_pred_occ);
        dim3 blockDim_occ(threads, 1, 1);

        // Time stamps
        std::chrono::time_point t3_occ = std::chrono::steady_clock::now();

        // Error check (for both distributions)
        gpuErrchk(cudaPeekAtLastError());

        // Time stamp
        std::chrono::time_point t4_info = std::chrono::steady_clock::now();

        /************************ KERNEL STARTS  ************************/
        // Launch kernel
        ::infoGainKernel<<<gridDim, blockDim>>>(d_out, _d_map, d_start, d_endpoints, d_max_range, d_num_endpoints, d_num_startpoints);

        // Time stamps
        std::chrono::time_point t5_info = std::chrono::steady_clock::now();
        std::chrono::time_point t4_occ = std::chrono::steady_clock::now();

        if (skipOcc)
        {
            printf("Skipping occlusion gain calculation \n");
        }
        else
        {
            // Launch kernel occlusion gain with dimensions
            ::occlusionGainKernel<<<gridDim_occ, blockDim_occ>>>(d_out_occ, _d_map, d_start_occ, d_endpoints_occ, d_max_range_occ, d_num_endpoints_occ, d_num_startpoints_occ, d_num_pred_occ);
        }

        /************************ KERNEL ENDS  ************************/

        // Time stamps
        std::chrono::time_point t5_occ = std::chrono::steady_clock::now();

        // Synchronize threads (better for debugging; slightly decreases performance)
        // gpuErrchk(cudaDeviceSynchronize());

        // Post processing info gain
        std::chrono::time_point t6_info = std::chrono::steady_clock::now();

        // Copy output from device to host
        gpuErrchk(cudaMemcpy(out, d_out, sizeof(double) * num_info * num_startpoints, cudaMemcpyDeviceToHost));

        // Copy output to Eigen matrix
        for (int blockIdx = 0; blockIdx < num_startpoints; blockIdx++)
        {
            results->row(blockIdx) = Eigen::Map<Eigen::VectorXd>(out + blockIdx * num_info, num_info);
            // printf("GPU result= %f \n", results->row(blockIdx).mean());
        }

        // Free memory on host
        free(out);

        // Free all the device memory
        cudaFree(d_out);
        cudaFree(d_endpoints);
        cudaFree(d_start);
        cudaFree(d_max_range);
        cudaFree(d_num_endpoints);
        cudaFree(d_num_startpoints);

        std::chrono::time_point t7_info = std::chrono::steady_clock::now();

        // Post processing occlusion gain
        std::chrono::time_point t6_occ = std::chrono::steady_clock::now();
        // Copy output from device to host
        gpuErrchk(cudaMemcpy(out_occ, d_out_occ, sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ, cudaMemcpyDeviceToHost));

        // Copy output to Eigen matrix

        for (int idx = 0; idx < num_pred_occ; idx++)
        {
            results_occ->row(idx) = Eigen::Map<Eigen::VectorXd>(out_occ + idx * num_occ, num_occ);
            // std::cout << "Row " << idx << " results: " << results_occ->row(idx) << std::endl;
        }

        // Free memory on host
        free(out_occ);

        // Free all the device memory
        cudaFree(d_out_occ);
        cudaFree(d_endpoints_occ);
        cudaFree(d_start_occ);
        cudaFree(d_max_range_occ);
        cudaFree(d_num_endpoints_occ);
        cudaFree(d_num_startpoints_occ);
        cudaFree(d_num_pred_occ);
        // Timing
        std::chrono::time_point t7_occ = std::chrono::steady_clock::now();

        // Clean all the children pointers from all nodes (important to avoid memory leaks)
        for (auto it = _h_children_map.begin(); it != _h_children_map.end(); ++it)
        {
            cudaFree(it->first);
        }

        std::chrono::time_point t_fin = std::chrono::steady_clock::now();

        // 1: map copy time; 2: copy info variables time 3: info kernel time; 4: cleanup time; 5: total data time (without kernel) 6: total time (incl. occlusion)
        *time_metrics << std::chrono::duration<float, std::chrono::seconds::period>(t2_info - t1).count(),
            std ::chrono::duration<float, std::chrono::seconds::period>(t3_info - t2_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t5_info - t4_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t7_info - t6_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_info - t2_info).count() + std::chrono::duration<float, std::chrono::seconds::period>(t7_info - t6_info).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t_fin - t1).count();

        // 1: map copy time; 2: copy occ variables time 3: occ kernel time; 4: cleanup occ time; 5: total occ data time (without kernel) 6: total time (incl. info)
        *time_metrics_occ << std::chrono::duration<float, std::chrono::seconds::period>(t2_info - t1).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_occ - t2_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t5_occ - t4_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t7_occ - t6_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_occ - t2_occ).count() + std::chrono::duration<float, std::chrono::seconds::period>(t7_occ - t6_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t_fin - t1).count();
    };

    void UFOMapGPU::calcOccGain(int num_occ, int num_pred_occ, int threads, std::vector<std::tuple<int, ufo::map::Point3, Eigen::AngleAxisd>> *start_points_occ,
                                std::map<int, std::vector<ufo::map::Point3>> *endpoints_occ, double max_range, Eigen::MatrixXd *results_occ,
                                Eigen::VectorXd *time_metrics_occ)
    {
        std::chrono::time_point t1 = std::chrono::steady_clock::now();

        // Initialize and update map
        if (_h_map == nullptr)
        {
            printf("Map not copied\n");
            return;
        }
        // Update the map on the GPU (including children pointers)
        updateMapGPU();

        // Time stamps
        std::chrono::time_point t2_occ = std::chrono::steady_clock::now();

        // Occlusion distribution preparation
        bool skipOcc = false;
        if (num_occ == 0)
        {
            skipOcc = true;
        }

        // host copy of output
        double *out_occ;

        // Convert the input to the correct format
        std::vector<ufo::map::Point3> endpoints_h_occ(start_points_occ->size() * num_occ);
        std::vector<ufo::map::Point3> startpoints_h_occ(start_points_occ->size());

        if (start_points_occ->size() == endpoints_occ->size())
        {

            for (int i = 0; i < start_points_occ->size(); i++)
            {
                startpoints_h_occ[i] = std::get<1>(start_points_occ->at(i));
                // Out of range error here
                std::vector<ufo::map::Point3> *ep = &endpoints_occ->at(std::get<0>(start_points_occ->at(i)));
                for (int j = 0; j < num_occ; j++)
                {

                    endpoints_h_occ[i * num_occ + j] = ep->at(j);
                }
            }
        }
        else
        {
            printf("start_points_occ and endpoints_occ size mismatch \n");
            skipOcc = true;
        }
        int num_startpoints_occ = startpoints_h_occ.size();

        // device copies of inputs and output
        double *d_out_occ;
        int *d_num_endpoints_occ, *d_num_startpoints_occ, *d_num_pred_occ;
        double *d_max_range_occ;
        ufo::map::Point3 *d_endpoints_occ;
        ufo::map::Point3 *d_start_occ;

        // Alloc space for device copies
        // one double for each endpoint
        gpuErrchk(cudaMalloc((void **)&d_out_occ, sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ));
        gpuErrchk(cudaMalloc((void **)&d_endpoints_occ, sizeof(ufo::map::Point3) * num_occ * num_startpoints_occ));
        gpuErrchk(cudaMalloc((void **)&d_start_occ, sizeof(ufo::map::Point3) * num_startpoints_occ));
        gpuErrchk(cudaMalloc((void **)&d_max_range_occ, sizeof(double)));    // alloc space for max_range double
        gpuErrchk(cudaMalloc((void **)&d_num_endpoints_occ, sizeof(int)));   // alloc space for num_endpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_startpoints_occ, sizeof(int))); // alloc space for num_startpoints int
        gpuErrchk(cudaMalloc((void **)&d_num_pred_occ, sizeof(int)));        // alloc space for num_pred int

        // Alloc space for host copies of info gain for all endpoints
        out_occ = (double *)malloc(sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ); // one double for each endpoint

        gpuErrchk(cudaMemcpy(d_max_range_occ, &max_range, sizeof(double), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_endpoints_occ, &num_occ, sizeof(int), cudaMemcpyHostToDevice));
        // copy num_endpoints to device
        gpuErrchk(cudaMemcpy(d_num_startpoints_occ, &num_startpoints_occ, sizeof(int), cudaMemcpyHostToDevice));
        // copy num_pred to device
        gpuErrchk(cudaMemcpy(d_num_pred_occ, &num_pred_occ, sizeof(int), cudaMemcpyHostToDevice));
        // copy startpoint to device
        gpuErrchk(cudaMemcpy(d_start_occ, startpoints_h_occ.data(), sizeof(ufo::map::Point3) * num_startpoints_occ, cudaMemcpyHostToDevice));
        // copy endpoints to device
        gpuErrchk(cudaMemcpy(d_endpoints_occ, endpoints_h_occ.data(), sizeof(ufo::map::Point3) * num_occ * num_startpoints_occ, cudaMemcpyHostToDevice));

        dim3 gridDim_occ(static_cast<int>(std::ceil(static_cast<double>(num_occ) / static_cast<double>(threads))), num_startpoints_occ, num_pred_occ);
        dim3 blockDim_occ(threads, 1, 1);

        // Time stamps
        std::chrono::time_point t3_occ = std::chrono::steady_clock::now();

        // Error check (for both distributions)
        gpuErrchk(cudaPeekAtLastError());

        // Time stamps
        std::chrono::time_point t4_occ = std::chrono::steady_clock::now();

        /************************ KERNEL STARTS  ************************/
        if (skipOcc)
        {
            printf("Skipping occlusion gain calculation \n");
        }
        else
        {
            ::occlusionGainKernel<<<gridDim_occ, blockDim_occ>>>(d_out_occ, _d_map, d_start_occ, d_endpoints_occ, d_max_range_occ, d_num_endpoints_occ, d_num_startpoints_occ, d_num_pred_occ);
        }
        /************************ KERNEL ENDS  ************************/
        // Time stamps
        std::chrono::time_point t5_occ = std::chrono::steady_clock::now();

        // Synchronize threads (better for debugging; slightly decreases performance)
        // gpuErrchk(cudaDeviceSynchronize());

        // Post processing occlusion gain
        std::chrono::time_point t6_occ = std::chrono::steady_clock::now();
        // Copy output from device to host
        // printf("Size of out_occ: %d \n",num_occ * num_startpoints_occ * num_pred_occ);
        gpuErrchk(cudaMemcpy(out_occ, d_out_occ, sizeof(double) * num_occ * num_startpoints_occ * num_pred_occ, cudaMemcpyDeviceToHost));

        // Copy output to Eigen matrix

        for (int idx = 0; idx < num_pred_occ; idx++)
        {
            results_occ->row(idx) = Eigen::Map<Eigen::VectorXd>(out_occ + idx * num_occ, num_occ);
            // std::cout << "Row " << idx << " results: " << results_occ->row(idx) << std::endl;
        }

        // Free memory on host
        free(out_occ);

        // Free all the device memory
        cudaFree(d_out_occ);
        cudaFree(d_endpoints_occ);
        cudaFree(d_start_occ);
        cudaFree(d_max_range_occ);
        cudaFree(d_num_endpoints_occ);
        cudaFree(d_num_startpoints_occ);
        cudaFree(d_num_pred_occ);
        // Timing
        std::chrono::time_point t7_occ = std::chrono::steady_clock::now();

        // Clean all the children pointers from all nodes (important to avoid memory leaks)
        for (auto it = _h_children_map.begin(); it != _h_children_map.end(); ++it)
        {
            cudaFree(it->first);
        }

        std::chrono::time_point t_fin = std::chrono::steady_clock::now();

        // 1: map copy time; 2: copy occ variables time 3: occ kernel time; 4: cleanup occ time; 5: total occ data time (without kernel) 6: total time (incl. info)
        *time_metrics_occ << std::chrono::duration<float, std::chrono::seconds::period>(t2_occ - t1).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_occ - t2_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t5_occ - t4_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t7_occ - t6_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t3_occ - t2_occ).count() + std::chrono::duration<float, std::chrono::seconds::period>(t7_occ - t6_occ).count(),
            std::chrono::duration<float, std::chrono::seconds::period>(t_fin - t1).count();
    };

    void UFOMapGPU::getChildrenPointer(INNER_NODE *root, ufo::map::OccupancyMapColor *map_h, int depth_cnt, int depth_max)
    { // Initialize variables
        void *d_children;
        INNER_NODE &inner_node = *root;

        // Recursion stop condition if we reached the max depth
        if (depth_cnt == depth_max - 1)
            return;

        // Check if the current node is a leaf node and return
        if (!map_h->hasChildren(inner_node))
        {
            return;
        }
        else // If the current node is an inner node, we want to get the children pointer
        {
            // Get the children pointer from the host (recursive call)
            for (int i = 0; i <= 7; i++)
            {
                INNER_NODE &child_node = static_cast<INNER_NODE &>(map_h->getChild(inner_node, 1, i));
                if (!child_node.is_leaf && child_node.children != nullptr)
                    getChildrenPointer(&child_node, map_h, depth_cnt + 1);
            }
            // Allocate memory on the device and copy the children pointer to the device
            gpuErrchk(cudaMalloc((void **)&d_children, sizeof(INNER_NODE) * 8));
            gpuErrchk(cudaMemcpy(d_children, inner_node.children, sizeof(INNER_NODE) * 8, cudaMemcpyHostToDevice));

            // Save the host and device pointer to the map
            _h_children_map[d_children] = root->children;

            // Set the children pointer of the current node to the device pointer for copying the whole map with device pointers
            root->children = d_children;
            return;
        }
    };

    void UFOMapGPU::setPointerBackToHost(INNER_NODE *root, ufo::map::OccupancyMapColor *map_h, int depth_cnt, int depth_max)
    {
        // Initialize variables
        INNER_NODE &inner_node = *root;

        // Recursion stop condition if we reached the max depth
        if (depth_cnt == depth_max - 1)
            return;

        // Check if the current node is a leaf node and return
        if (!map_h->hasChildren(inner_node))
        {
            return;
        }
        else // If the current node is an inner node, we want to get the children pointer back to the host
        {
            // Get the children pointer from the device and set to host pointer
            root->children = _h_children_map[root->children];

            // Get the children pointer from the device and set to host pointer (recursive call)
            for (int i = 0; i <= 7; i++)
            {
                INNER_NODE &child_node = static_cast<INNER_NODE &>(map_h->getChild(inner_node, 1, i));
                if (!child_node.is_leaf && child_node.children != nullptr)
                    setPointerBackToHost(&child_node, map_h, depth_cnt + 1);
            }
        }
    };

} // namespace ufomapGPU
