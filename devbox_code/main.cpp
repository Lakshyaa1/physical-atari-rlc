// Copyright 2026 Keen Technologies, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "imgui/imgui.h"
#include "imgui/imgui_impl_sdl2.h"
#include "imgui/imgui_impl_sdlrenderer2.h"
#include "imgui/imgui_theme.h"
#include <chrono>
#include <fstream>

#include <iostream>
#include <vector>
#include <csignal>
#include <cerrno>
#include <cstring>
#include <unordered_map>

#include <SDL2/SDL.h>
#include <mutex>
#include <thread>
#include <memory>
#include "src/atari/ale_interface.hpp"
#include "include/json.hpp"
#include "include/joystick_input.h"


// Screen Settings
int screen_width = 1280;
int screen_height = 720;

// SDL Core Components
SDL_Window* window;
SDL_Renderer* renderer;
SDL_Texture* game_screen;

// Frame Timing
double fps_estimate = 60;

// ALE Environment and Configuration
ale::ALEInterface* env;
ale::Action current_action = ale::PLAYER_A_NOOP;

// Game State and Rendering
std::vector<unsigned char> input_image;

// Synchronization
std::mutex m;

// Episodic tracking
std::vector<int> episodic_scores;
std::vector<int> episode_lengths;
int current_episode_score = 0;
std::string experiment_name;
std::string game_name_global;
std::chrono::system_clock::time_point start_timestamp;
bool shutdown_requested = false;

// Usage text for the command line.
static void print_usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " <RomDirectory> <game_name> [results_filename] [options]\n"
        << "\n"
        << "Options:\n"
        << "  --input=serial[:DEV]  read the CX40 switches from an MCU over USB CDC\n"
        << "                        (default device /dev/ttyUSB0)\n"
        << "  --input=keyboard      arrow keys + space; no robot hardware needed\n"
#ifdef DEVBOX_HAVE_GPIOD
        << "  --input=gpio[:CHIP]   read the switches directly on Raspberry Pi GPIO\n"
        << "                        (default chip /dev/gpiochip0)\n"
#endif
        << "  --baud=N              serial baud rate (default 115200)\n"
        << "  --frac=F              render scale; design is 1280x720, so F=1.5\n"
        << "                        exactly fills a 1920x1080 screen.\n"
        << "                        'auto' (default) fits the current display.\n"
        << "  --windowed            run in a window instead of fullscreen\n"
        << "  --dump-frame=PATH     save one rendered frame to PATH (.bmp) and exit.\n"
        << "                        Use this to verify the AprilTags and game crop\n"
        << "                        without needing the camera.\n"
        << "  --dump-after=N        step N frames before dumping (default 60)\n"
        << "  --fps=N               cap the emulator at N frames/sec (default 60,\n"
        << "                        the Atari's native rate). 0 = rely on vsync only,\n"
        << "                        which runs the game at the display's refresh rate.\n"
        << std::endl;
}

// Signal handler for graceful shutdown
void signal_handler(int signum)
{
    std::cout << "\n[PhysicalALE] Received signal " << signum << ", shutting down gracefully..." << std::endl;
    m.lock();
    shutdown_requested = true;
    m.unlock();
}

// Function to get ISO 8601 timestamp string
std::string get_iso_timestamp(const std::chrono::system_clock::time_point& tp)
{
    auto time_t_tp = std::chrono::system_clock::to_time_t(tp);
    std::tm tm = *std::gmtime(&time_t_tp);
    char buffer[100];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buffer);
}

std::string get_filename_timestamp(const std::chrono::system_clock::time_point& tp)
{
    auto time_t_tp = std::chrono::system_clock::to_time_t(tp);
    std::tm tm = *std::gmtime(&time_t_tp);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &tm);
    return std::string(buffer);
}

// Function to save results to JSON file
void save_results_to_json()
{
    try
    {
        std::cout << "[PhysicalALE] Saving results to JSON file..." << std::endl;
        
        // Create end timestamp
        auto end_timestamp = std::chrono::system_clock::now();
        
        // Build JSON document
        nlohmann::json j;
        j["experiment_name"] = experiment_name;
        j["game"] = game_name_global;
        j["episodic_scores"] = episodic_scores;
        j["episode_lengths"] = episode_lengths;
        j["start_timestamp"] = get_iso_timestamp(start_timestamp);
        j["end_timestamp"] = get_iso_timestamp(end_timestamp);
        
        std::string filename = experiment_name.empty()
            ? game_name_global + "_" + get_filename_timestamp(end_timestamp) + ".json"
            : experiment_name + ".json";
        std::ofstream output_file(filename);
        if (!output_file.is_open())
        {
            std::cerr << "[PhysicalALE] Error: Could not open output file " << filename << std::endl;
            return;
        }
        
        output_file << j.dump(4);  // Pretty print with 4-space indent
        output_file.close();
        
        std::cout << "[PhysicalALE] Results saved to " << filename << std::endl;
        std::cout << "[PhysicalALE] Total episodes: " << episodic_scores.size() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "[PhysicalALE] Error saving results to JSON: " << e.what() << std::endl;
    }
}

// Function to generate AprilTag
void GenerateAprilTag(unsigned char* aprilTag, const std::vector<int>& indices)
{
    for (int i = 0; i < 10 * 10 * 4; i++)
    {
        aprilTag[i] = 0;
    }
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 10; j++)
        {
            if (i == 0 || j == 0 || i == 9 || j == 9)
            {
                aprilTag[i * 10 * 4 + j * 4 + 0] = 255;
                aprilTag[i * 10 * 4 + j * 4 + 1] = 255;
                aprilTag[i * 10 * 4 + j * 4 + 2] = 255;
                aprilTag[i * 10 * 4 + j * 4 + 3] = 255;
            }
        }
    }
    for (auto& ind : indices)
    {
        aprilTag[(ind - 1) * 4 + 0] = 255;
        aprilTag[(ind - 1) * 4 + 1] = 255;
        aprilTag[(ind - 1) * 4 + 2] = 255;
        aprilTag[(ind - 1) * 4 + 3] = 255;
    }
}


int main(int argc, char** argv)
{
    // Record start timestamp
    start_timestamp = std::chrono::system_clock::now();
    
    // frac scales the whole 1280x720 design layout to the physical panel. The
    // Pi devbox used 0.625 to fit its 800x480 display. Everything (game rect,
    // corner tags, reward tags) is scaled by it uniformly, and the agent's
    // homography maps the detected corner tags back into design space — so
    // changing frac is safe end to end and simply buys the camera more pixels.
    // -1 means "fit the current display", resolved after SDL_Init below.
    float frac = -1.0f;

    std::string input_spec = "serial";
    int serial_baud = 115200;
    bool windowed = false;
    std::string dump_frame_path;
    int dump_after_steps = 60;
    // The Atari runs at 60 Hz. Upstream relied solely on SDL's vsync to pace
    // the loop, which is safe on the Raspberry Pi devbox's fixed 60 Hz panel
    // but not on a laptop: dock it to a 144 Hz monitor and the emulator --
    // and therefore the game, the reward rate, and every latency figure --
    // runs 2.4x too fast, silently. Cap it explicitly. 0 = vsync only.
    int target_fps = 60;

    // Positional: <RomDirectory> <game_name> [results_filename]
    // Anything starting with "--" is an option and may appear anywhere after.
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.rfind("--", 0) != 0)
        {
            positional.push_back(arg);
            continue;
        }

        if (arg == "--help" || arg == "-h")
        {
            print_usage(argv[0]);
            return 0;
        }
        else if (arg.rfind("--input=", 0) == 0)
        {
            input_spec = arg.substr(8);
        }
        else if (arg.rfind("--baud=", 0) == 0)
        {
            serial_baud = std::atoi(arg.substr(7).c_str());
        }
        else if (arg.rfind("--frac=", 0) == 0)
        {
            std::string v = arg.substr(7);
            frac = (v == "auto") ? -1.0f : std::atof(v.c_str());
        }
        else if (arg == "--windowed")
        {
            windowed = true;
        }
        else if (arg.rfind("--dump-frame=", 0) == 0)
        {
            dump_frame_path = arg.substr(13);
        }
        else if (arg.rfind("--dump-after=", 0) == 0)
        {
            dump_after_steps = std::atoi(arg.substr(13).c_str());
        }
        else if (arg.rfind("--fps=", 0) == 0)
        {
            target_fps = std::atoi(arg.substr(6).c_str());
        }
        else
        {
            std::cerr << "Error: unknown option '" << arg << "'" << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    if (positional.size() < 2)
    {
        std::cerr << "Error: RomDirectory and game_name are required" << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    std::string directory = positional[0];
    std::string game_name = positional[1];
    game_name_global = game_name;
    experiment_name = (positional.size() >= 3) ? positional[2] : "";

    std::unordered_map<std::string, bool> to_render_flags_map;
    to_render_flags_map["corner_tags"] = true;

    std::cout << "Game: " << game_name << ", Experiment: " << experiment_name << std::endl;

    // Register signal handler for graceful shutdown
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    env = new ale::ALEInterface();
    env->setFloat("repeat_action_probability", 0.0);
    env->setInt("frame_skip", 1);
    env->setBool("truncate_on_loss_of_life", false);
    // Ensure directory has trailing slash for proper path joining
    std::string rom_path = directory;
    if (!rom_path.empty() && rom_path.back() != '/') {
        rom_path += "/";
    }
    rom_path += game_name + ".bin";
    env->loadROM(rom_path);

    env->reset_game();
    std::cout << "Env reset complete\n";
    env->act(ale::PLAYER_A_NOOP);
    SDL_Init(SDL_INIT_VIDEO);
    SDL_DisplayMode DM;
    SDL_GetCurrentDisplayMode(0, &DM);

    // Resolve "auto": scale the 1280x720 design as large as the display allows
    // without cropping. A 1920x1080 panel gives exactly 1.5.
    if (frac <= 0.0f)
    {
        const float fit_w = static_cast<float>(DM.w) / static_cast<float>(screen_width);
        const float fit_h = static_cast<float>(DM.h) / static_cast<float>(screen_height);
        frac = (fit_w < fit_h) ? fit_w : fit_h;
        std::cout << "[devbox] display " << DM.w << "x" << DM.h << " -> auto frac " << frac << std::endl;
    }
    const int content_w = static_cast<int>(screen_width * frac);
    const int content_h = static_cast<int>(screen_height * frac);
    std::cout << "[devbox] rendering " << content_w << "x" << content_h
              << " (frac " << frac << ")" << std::endl;

    // Fullscreen-desktop keeps the current video mode (no mode switch, which is
    // both faster to start and far friendlier under Wayland/XWayland).
    auto window_flags = static_cast<SDL_WindowFlags>(
        windowed ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
    window = SDL_CreateWindow("Atari 2600", SDL_WINDOWPOS_CENTERED,
                              SDL_WINDOWPOS_CENTERED,
                              windowed ? content_w : screen_width,
                              windowed ? content_h : screen_height,
                              window_flags);

    renderer = SDL_CreateRenderer(
        window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr)
    {
        SDL_Log("Error creating SDL_Renderer!");
        return 0;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    ImGuiTheme::ApplyTheme(ImGuiTheme::ImGuiTheme_GrayVariations);

    bool done = false;
    int pitch2 = 0;
    void* pixels2;

    std::vector<std::vector<int>> indices = {};
    std::vector<int> tag0 = {23, 24, 26, 28, 34, 35, 36, 38, 44, 45, 53, 55, 64, 66, 67, 76};
    std::vector<int> tag1 = {23, 24, 26, 27, 34, 36, 37, 38, 43, 44, 45, 46, 54, 55, 63, 65, 66, 68, 75, 78};
    std::vector<int> tag2 = {23, 24, 26, 27, 28, 34, 37, 43, 55, 58, 66, 75, 76, 77};
    std::vector<int> tag3 = {23, 24, 25, 28, 36, 37, 38, 43, 46, 47, 48, 53, 55, 58, 63, 64, 67, 74, 75};
    std::vector<int> tag4 = {23, 24, 25, 27, 33, 34, 35, 36, 43, 45, 46, 47, 48, 55, 57, 63, 73, 77};
    std::vector<int> tag5 = {23, 24, 25, 26, 33, 34, 38, 43, 44, 46, 47, 53, 55, 57, 58, 65, 66, 67, 73, 75, 76};
    std::vector<int> tag6 = {28, 34, 36, 37, 43, 45, 48, 54, 55, 56, 58, 67, 76, 78};
    std::vector<int> tag7 = {26, 36, 37, 44, 46, 53, 55, 56, 57, 66, 67, 68, 74, 76};
    std::vector<int> tag8 = {25, 33, 35, 37, 38, 46, 47, 48, 54, 55, 56, 57, 58, 63, 64, 65, 67, 73, 75, 76, 78};
    std::vector<int> tag9 = {25, 28, 33, 36, 38, 43, 45, 47, 48, 54, 64, 68, 73, 74, 77};
    std::vector<int> tag10 = {25, 26, 28, 35, 36, 37, 38, 43, 44, 45, 47, 54, 58, 63, 65, 66, 67, 76, 77};
    std::vector<int> tag11 = {25, 26, 27, 28, 33, 34, 35, 36, 37, 38, 43, 46, 48, 53, 57, 63, 64, 67, 68, 74, 76, 78};
    std::vector<int> tag12 = {24, 33, 34, 35, 37, 45, 48, 54, 57, 58, 65, 67, 74, 75, 77};
    std::vector<int> tag13 = {24, 28, 33, 34, 36, 43, 45, 46, 48, 57, 58, 63, 68, 74, 75, 76, 77, 78};
    std::vector<int> tag14 = {24, 27, 28, 33, 35, 38, 43, 44, 46, 53, 56, 65, 66, 67, 68, 73, 75, 78};
    std::vector<int> tag15 = {24, 26, 33, 36, 44, 45, 54, 56, 63, 66, 67, 73, 75, 76, 77};
    std::vector<int> tag16 = {24, 26, 27, 34, 35, 38, 44, 45, 46, 47, 48, 53, 54, 56, 58, 64, 66, 73, 74, 75};
    std::vector<int> tag17 = {24, 25, 27, 36, 37, 43, 45, 46, 47, 53, 54, 56, 57, 63, 64, 68, 75, 76};
    std::vector<int> tag18 = {24, 25, 27, 33, 34, 35, 36, 37, 44, 47, 53, 56, 57, 58, 65, 73, 74, 78};
    std::vector<int> tag19 = {24,25, 26, 27, 28, 33, 37, 38, 46, 48, 54, 55, 63, 64, 65, 67, 68, 73, 75, 77};
    indices = {
        tag0, tag1, tag2, tag3, tag4, tag5, tag6, tag7, tag8, tag9, tag10, tag11, tag12, tag13, tag14, tag15, tag16,
        tag17, tag18, tag19
    };


    std::vector<SDL_Texture*> aprilTagTextures;
    game_screen = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
                                    static_cast<int>(env->getWidth()),
                                    static_cast<int>(env->getHeight()));
    for (int i = 0; i < 36; i++)
    {
        aprilTagTextures.push_back(SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR888, SDL_TEXTUREACCESS_STREAMING,
                                                     10,
                                                     10));
    }

    std::vector<unsigned char> aprilTag(10 * 10 * 4, 0);
    for (int i = 0; i < indices.size(); i++)
    {
        GenerateAprilTag(aprilTag.data(), indices[i]);
        if (SDL_LockTexture(aprilTagTextures[i], nullptr, &pixels2, &pitch2) < 0)
        {
            std::cerr << "Failed to lock texture: " << SDL_GetError() << std::endl;
        };
        std::memcpy(pixels2, aprilTag.data(), pitch2 * 10);
        SDL_UnlockTexture(aprilTagTextures[i]);
    }


    auto old_time = std::chrono::system_clock::now();
    auto next_frame_deadline = std::chrono::steady_clock::now();

    int step_counter = 0;
    int last_nonzero_reward_tag = 10;  // Starts at tag 10 (init condition)
    int change_indicator_tag = 15;     // Cycles through 15, 16, 17
    int current_reward = 0;            // Store reward from each step
    bool show_fps = false;


    // Build the joystick input source from --input=...
    std::unique_ptr<InputSource> input;
    {
        std::string kind = input_spec;
        std::string target;
        const size_t colon = input_spec.find(':');
        if (colon != std::string::npos)
        {
            kind = input_spec.substr(0, colon);
            target = input_spec.substr(colon + 1);
        }

        if (kind == "keyboard")
        {
            input = std::make_unique<KeyboardInput>();
            std::cout << "[devbox] input: keyboard (arrows + space)" << std::endl;
        }
        else if (kind == "serial")
        {
            auto serial = std::make_unique<SerialInput>();
            if (!serial->open_port(target.empty() ? "/dev/ttyUSB0" : target, serial_baud))
            {
                std::cerr << "Failed to open the joystick serial port. Pass "
                             "--input=serial:/dev/ttyACM0 (or similar), or "
                             "--input=keyboard to run without robot hardware."
                          << std::endl;
                return 1;
            }
            input = std::move(serial);
        }
#ifdef DEVBOX_HAVE_GPIOD
        else if (kind == "gpio")
        {
            auto gpio = std::make_unique<GpioInput>();
            if (!gpio->open_chip(target.empty() ? "/dev/gpiochip0" : target.c_str()))
            {
                std::cerr << "Failed to request GPIO lines." << std::endl;
                return 1;
            }
            input = std::move(gpio);
        }
#endif
        else
        {
            std::cerr << "Error: unknown input source '" << kind << "'" << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }
    ButtonState buttons;

    while (!done)
    {
        // Check for shutdown request
        m.lock();
        if (shutdown_requested)
        {
            done = true;
            m.unlock();
            break;
        }
        m.unlock();
        
        // Read the CX40 switches. Deliberately done immediately before
        // env->act() below so the emulator always steps on the freshest input.
        // (The keyboard source reads SDL's cached key state, which is refreshed
        // by the SDL_PollEvent loop further down — so it lags by at most one
        // frame. That path is for bring-up only, not for robot runs.)
        input->poll(buttons);
        const int up = buttons.up;
        const int down = buttons.down;
        const int left = buttons.left;
        const int right = buttons.right;
        const int fire = buttons.fire;

        if (up == 0 && right == 0 && left == 0 && fire == 0 && down == 0) current_action = ale::PLAYER_A_NOOP;
        else if (up == 1 && right == 0 && left == 0 && fire == 0 && down == 0) current_action = ale::PLAYER_A_UP;
        else if (up == 0 && right == 1 && left == 0 && fire == 0 && down == 0) current_action = ale::PLAYER_A_RIGHT;
        else if (up == 0 && right == 0 && left == 1 && fire == 0 && down == 0) current_action = ale::PLAYER_A_LEFT;
        else if (up == 0 && right == 0 && left == 0 && fire == 1 && down == 0) current_action = ale::PLAYER_A_FIRE;
        else if (up == 0 && right == 0 && left == 0 && fire == 0 && down == 1) current_action = ale::PLAYER_A_DOWN;
        else if (up == 1 && right == 1 && left == 0 && fire == 0 && down == 0) current_action = ale::PLAYER_A_UPRIGHT;
        else if (up == 1 && right == 0 && left == 1 && fire == 0 && down == 0) current_action = ale::PLAYER_A_UPLEFT;
        else if (up == 0 && right == 1 && left == 0 && fire == 0 && down == 1) current_action = ale::PLAYER_A_DOWNRIGHT;
        else if (up == 0 && right == 0 && left == 1 && fire == 0 && down == 1) current_action = ale::PLAYER_A_DOWNLEFT;
        else if (up == 1 && right == 0 && left == 0 && fire == 1 && down == 0) current_action = ale::PLAYER_A_UPFIRE;
        else if (up == 0 && right == 1 && left == 0 && fire == 1 && down == 0) current_action = ale::PLAYER_A_RIGHTFIRE;
        else if (up == 0 && right == 0 && left == 1 && fire == 1 && down == 0) current_action = ale::PLAYER_A_LEFTFIRE;
        else if (up == 0 && right == 0 && left == 0 && fire == 1 && down == 1) current_action = ale::PLAYER_A_DOWNFIRE;
        else if (up == 1 && right == 1 && left == 0 && fire == 1 && down == 0)
            current_action =
                ale::PLAYER_A_UPRIGHTFIRE;
        else if (up == 1 && right == 0 && left == 1 && fire == 1 && down == 0)
            current_action =
                ale::PLAYER_A_UPLEFTFIRE;
        else if (up == 0 && right == 0 && left == 1 && fire == 1 && down == 1)
            current_action =
                ale::PLAYER_A_DOWNLEFTFIRE;
        else if (up == 0 && right == 1 && left == 0 && fire == 1 && down == 1)
            current_action =
                ale::PLAYER_A_DOWNRIGHTFIRE;

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        SDL_Event event;

        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                done = true;
            if (event.type == SDL_WINDOWEVENT &&
                event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }
        step_counter++;
        current_reward = env->act(current_action);
        
        // Accumulate reward for current episode
        current_episode_score += current_reward;
        
        // Update reward tags based on current reward
        if (current_reward > 0)
        {
            last_nonzero_reward_tag = 11;
            change_indicator_tag = (change_indicator_tag == 15) ? 16 : (change_indicator_tag == 16) ? 17 : 15;
        }
        else if (current_reward < 0)
        {
            last_nonzero_reward_tag = 12;
            change_indicator_tag = (change_indicator_tag == 15) ? 16 : (change_indicator_tag == 16) ? 17 : 15;
        }
        // If current_reward == 0, keep last_nonzero_reward_tag unchanged and don't cycle change_indicator_tag
        
        if (env->game_over())
        {
            // Save episode data
            episodic_scores.push_back(current_episode_score);
            episode_lengths.push_back(step_counter);
            std::cout << "Episode ended - Score: " << current_episode_score << ", Length: " << step_counter << std::endl;
            
            env->reset_game();
            step_counter = 0;
            current_episode_score = 0;
            last_nonzero_reward_tag = 10;
            change_indicator_tag = 15;
        }
        if(step_counter > 27000)
        {
            std::cout << "Resetting game after 27000 steps" << std::endl;
            
            // Save episode data
            episodic_scores.push_back(current_episode_score);
            episode_lengths.push_back(step_counter);
            std::cout << "Episode ended - Score: " << current_episode_score << ", Length: " << step_counter << std::endl;
            
            env->reset_game();
            step_counter = 0;
            current_episode_score = 0;
            last_nonzero_reward_tag = 10;
            change_indicator_tag = 15;
        }
        env->getScreenRGB(input_image);

        if (ImGui::IsKeyPressed(ImGuiKey_Q, false))
        {
            done = true;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_T, false))
        {
            to_render_flags_map["corner_tags"] = !to_render_flags_map["corner_tags"];
        }
        if (ImGui::IsKeyPressed(ImGuiKey_F, false))
        {
            show_fps = !show_fps;
        }

        if (show_fps)
        {
            const float tag_size = 120.f * frac;
            const float left_tag_center_x = tag_size * 0.5f;
            const float left_tag_gap_center_y = (tag_size + (screen_height - 120) * frac) * 0.5f;
            ImGui::SetNextWindowPos(
                ImVec2(left_tag_center_x, left_tag_gap_center_y),
                ImGuiCond_Always,
                ImVec2(0.5f, 0.5f));
            ImGui::SetNextWindowBgAlpha(0.5f);
            ImGui::Begin("FPS", nullptr,
                         ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoInputs);
            ImGui::Text("FPS: %.1f", fps_estimate);
            ImGui::End();
        }

        ImGui::Render();
        SDL_RenderSetScale(renderer, 1,
                           1);
        ImVec4 clear_color = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);

        SDL_SetRenderDrawColor(
            renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255),
            (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
        SDL_RenderClear(renderer);

        void* pixels;
        int pitch;
        SDL_Rect dst;

        if (SDL_LockTexture(game_screen, nullptr, &pixels, &pitch) < 0)
        {
            std::cerr << "Failed to lock texture: " << SDL_GetError() << std::endl;
        }
        std::memcpy(pixels, input_image.data(), pitch * env->getHeight());
        SDL_UnlockTexture(game_screen);
        dst = {int(193 * frac), int(44 * frac), int(930 * frac), int(647 * frac)};
        SDL_RenderCopy(renderer, game_screen, nullptr, &dst);


        // We make the four corner apriltags
        int size_april_tag = 120;
        if (to_render_flags_map["corner_tags"])
        {
            dst = {0, 0, int(size_april_tag * frac), int(size_april_tag * frac)};
            SDL_RenderCopy(renderer, aprilTagTextures[0], nullptr, &dst);

            dst = {
                int(screen_width * frac) - int(frac * size_april_tag), 0, int(frac * size_april_tag),
                int(frac * size_april_tag)
            };
            SDL_RenderCopy(renderer, aprilTagTextures[1], nullptr, &dst);

            dst = {
                int(screen_width * frac) - int(frac * size_april_tag), int((screen_height - size_april_tag) * frac),
                int(frac * size_april_tag), int(frac * size_april_tag)
            };
            SDL_RenderCopy(renderer, aprilTagTextures[3], nullptr, &dst);

            dst = {0, int((screen_height - size_april_tag) * frac), int(frac * size_april_tag), int(frac * size_april_tag)};
            SDL_RenderCopy(renderer, aprilTagTextures[2], nullptr, &dst);
        }

        // Render reward AprilTags on the right side
        // Two tags vertically stacked, same size as corner tags (120x120)
        int reward_tag_size = 120;
        
        // Calculate vertical position to center both tags between top_right and bottom_right
        float available_space_start = size_april_tag * frac;
        float available_space_end = (screen_height - size_april_tag) * frac;
        float available_space_center = (available_space_start + available_space_end) / 2.0f;
        
        // Total height of 2 tags
        float total_tags_height = 2 * reward_tag_size * frac;
        float start_y = available_space_center - total_tags_height / 2.0f;
        
        // Center horizontally between corner tags' left edge and screen right edge
        float corner_tag_left = screen_width * frac - size_april_tag * frac;
        float screen_right = screen_width * frac;
        float center_x = (corner_tag_left + screen_right) / 2.0f;
        int right_x = int(center_x - reward_tag_size * frac / 2.0f);
        
        // Render reward value tag (top): tag 10 (init), 11 (positive), or 12 (negative)
        dst = {right_x, int(start_y), int(reward_tag_size * frac), int(reward_tag_size * frac)};
        SDL_RenderCopy(renderer, aprilTagTextures[last_nonzero_reward_tag], nullptr, &dst);
        
        // Render change indicator tag (bottom): cycles through 15, 16, 17
        float bottom_tag_y = start_y + reward_tag_size * frac;
        dst = {right_x, int(bottom_tag_y), int(reward_tag_size * frac), int(reward_tag_size * frac)};
        SDL_RenderCopy(renderer, aprilTagTextures[change_indicator_tag], nullptr, &dst);


        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());

        // Optional single-frame capture. Read the back buffer *before* present:
        // afterwards its contents are undefined. Lets you confirm the tag layout
        // and game crop offline, with no camera and no agent machine.
        if (!dump_frame_path.empty() && step_counter >= dump_after_steps)
        {
            int out_w = 0, out_h = 0;
            SDL_GetRendererOutputSize(renderer, &out_w, &out_h);
            SDL_Surface* shot =
                SDL_CreateRGBSurfaceWithFormat(0, out_w, out_h, 32, SDL_PIXELFORMAT_ARGB8888);
            if (shot != nullptr &&
                SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888, shot->pixels,
                                     shot->pitch) == 0 &&
                SDL_SaveBMP(shot, dump_frame_path.c_str()) == 0)
            {
                std::cout << "[devbox] wrote frame dump " << dump_frame_path << " (" << out_w << "x"
                          << out_h << ")" << std::endl;
            }
            else
            {
                std::cerr << "[devbox] frame dump failed: " << SDL_GetError() << std::endl;
            }
            if (shot != nullptr)
            {
                SDL_FreeSurface(shot);
            }
            done = true;
        }

        SDL_RenderPresent(renderer);

        // Hold the emulator to its native rate regardless of what the display
        // is doing. Advancing the deadline by a fixed step (rather than from
        // "now") keeps the average exact; if we fall behind, reset to now so a
        // hiccup does not build up a debt that then runs the game fast.
        if (target_fps > 0)
        {
            next_frame_deadline += std::chrono::microseconds(1000000 / target_fps);
            const auto now = std::chrono::steady_clock::now();
            if (next_frame_deadline > now)
            {
                std::this_thread::sleep_until(next_frame_deadline);
            }
            else
            {
                next_frame_deadline = now;
            }
        }

        auto current_time = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds =
            current_time - old_time;
        old_time = current_time;
        fps_estimate = fps_estimate * 0.99 + 0.01 * 1.0 / (elapsed_seconds.count());
    }

    // Save results to JSON file before exit
    std::cout << "[PhysicalALE] Cleaning up and saving results..." << std::endl;
    save_results_to_json();
    
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    input.reset(); // closes the serial port / releases GPIO lines
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    std::cout << "[PhysicalALE] Shutdown complete." << std::endl;
}
