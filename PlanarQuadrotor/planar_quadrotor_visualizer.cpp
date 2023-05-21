#include "planar_quadrotor_visualizer.h"
#include <SDL.h>
#include <cmath>
#include <iostream>

float angle = 0.0;

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor *quadrotor_ptr): quadrotor_ptr(quadrotor_ptr){};


/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer> &gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    int screen_width = 1280;
    int screen_height = 720;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);
    int q_x_screen = q_x + (screen_width/2);
    int q_y_screen = -q_y + (screen_height/2) + 29;
    angle = angle + 60;

    // "Kadlub" drona
    SDL_Rect body = {q_x_screen - 85, q_y_screen - 30, 170, 9};
    SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xA5, 0x2A, 0x2A); 
    SDL_RenderFillRect(gRenderer.get(), &body);
    //Pierwsze smiglo
    SDL_Rect prop1 = { q_x_screen - 95, q_y_screen - 51 , 65, 5 };
    //Drugie smiglo
    SDL_Rect prop2 = { q_x_screen + 35, q_y_screen - 51 , 65, 5 };
    
    SDL_Surface* buffer = SDL_LoadBMP("prop.bmp");
	if ( !buffer ) {
		std::cout << "Error loading image prop.bmp: " << SDL_GetError() << std::endl;
    }

    SDL_Texture* prop_texture;
	prop_texture = SDL_CreateTextureFromSurface(gRenderer.get(),buffer);
	SDL_FreeSurface(buffer);
	buffer = NULL;
	if ( !prop_texture ) {
		std::cout << "Error creating texture: " << SDL_GetError() << std::endl;
	}
    SDL_RenderCopyEx(gRenderer.get(), prop_texture, nullptr, &prop1, angle, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), prop_texture, NULL, &prop2, angle, NULL, SDL_FLIP_NONE);
     
     //Podporki
    SDL_Rect cant1 = {q_x_screen - 65, q_y_screen - 46 , 4, 17};
    SDL_SetRenderDrawColor(gRenderer.get(), 47, 61, 93, 30); 
    SDL_RenderFillRect(gRenderer.get(), &cant1);

    SDL_Rect cant2 = {q_x_screen + 65, q_y_screen - 46 , 4, 17};
    SDL_SetRenderDrawColor(gRenderer.get(), 47, 61, 93, 30); 
    SDL_RenderFillRect(gRenderer.get(), &cant2);

    SDL_RenderPresent(gRenderer.get());
    
}
