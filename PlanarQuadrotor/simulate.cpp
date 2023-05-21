/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <matplot/matplot.h>
#include <random> 

Eigen::MatrixXf LQR(PlanarQuadrotor &quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 4e-3, 4e-3, 4e2, 8e-3, 4.5e-2, 2 / 2 / M_PI;
    R.row(0) << 3e1, 7;
    R.row(1) << 7, 3e1;


    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;
    
    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor &quadrotor, const Eigen::MatrixXf &K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void plotGraphs(const std::vector<float>& x_history, const std::vector<float>& y_history, const std::vector<float>& theta_history);

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    /**
     ZROBIONE ++++++++++++++++++++
     * TODO: Extend simulation
     * 1. Set goal state of the mouse when clicking left mouse button (transform the coordinates to the quadrotor world! see visualizer TODO list)
     *    [x, y, 0, 0, 0, 0]
     * 2. Update PlanarQuadrotor from simulation when goal is changed
    */
   
     Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
     
    std::random_device random;
    std::mt19937 gen(random());
    
    //zakresy losowosci x i y
    
    std::uniform_int_distribution<int> random_x1(-640, -500);
    std::uniform_int_distribution<int> random_x2(500, 640);
    std::uniform_int_distribution<int> random_y1(-320, -250);
    std::uniform_int_distribution<int> random_y2(250, 320);

   
   //2 razy wybor z rownym prawdopodobienstwem
    
    std::uniform_int_distribution<int> choice1(0, 1);
    if (choice1(gen) == 0)
        initial_state[1] = random_y1(gen);  // Losuj y z pierwszego zakresu
    else
        initial_state[1] = random_y2(gen);  // Losuj y z drugiego zakresu
     
    std::uniform_int_distribution<int> choice2(0, 1);
    
     if (choice2(gen) == 0)
        initial_state[0] = random_x1(gen);  // Losuj y z pierwszego zakresu
    else
        initial_state[0] = random_x2(gen);  // Losuj y z drugiego zakresu

    std:: cout << "Starting from : " << initial_state[0] << ", " << initial_state[1] << std::endl;
    
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);
   
    /**
     ZROBIONE ++++++++++++++++++++
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
   
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << -1, 7, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.01;
    Eigen::MatrixXf K = LQR(quadrotor, dt);
    Eigen::Vector2f input = Eigen::Vector2f::Zero(2);

    /**
     ZROBIONE ++++++++++++++++++++
     * TODO: Plot x, y, theta over time
     * 1. Update x, y, theta history vectors to store trajectory of the quadrotor
     * 2. Plot trajectory using matplot++ when key 'p' is clicked
    */
    std::vector<float> x_history;
    std::vector<float> y_history;
    std::vector<float> theta_history;

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float delay;
        int x, y;
        Eigen::VectorXf state = Eigen::VectorXf::Zero(6);

        while (!quit)
        {
            //pobieranie wspolrzednych quadrotora i wpisywanie ich do wektorow historii zmian x,y,th
            state = quadrotor.GetState();  
            x_history.push_back(state[0]);
            y_history.push_back(state[1]);
            theta_history.push_back(state[2]);
            
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.type)
                {
                    
                    SDL_GetMouseState(&x, &y);
                    float quadrotorX = (x - SCREEN_WIDTH / 2);
                    float quadrotorY = -(y - SCREEN_HEIGHT / 2);
                    std::cout << "Target for the quadrotor : (" << quadrotorX << ", " << quadrotorY << ")" << std::endl;
                    goal_state << quadrotorX, quadrotorY, 0, 0, 0, 0;
                    quadrotor.SetGoal(goal_state);
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p)
                {
                    plotGraphs(x_history, y_history, theta_history);
                }
                
            }

            SDL_Delay((int) dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}

void plotGraphs(const std::vector<float>& x_history, const std::vector<float>& y_history, const std::vector<float>& theta_history)
{
                    
      //x             
    matplot::figure();
    matplot::plot(x_history);
    matplot::title("Change of X variable");
    matplot::xlabel("time");
    matplot::ylabel("x");
        //y                         
    matplot::figure();
    matplot::plot(y_history);
    matplot::title("Change of Y variable");
    matplot::xlabel("time");
    matplot::ylabel("y");
        //th               
    matplot::figure();
    matplot::plot(theta_history);
    matplot::title("Change of Theta angle");
    matplot::xlabel("time");
    matplot::ylabel("theta");
                    
    }

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}
