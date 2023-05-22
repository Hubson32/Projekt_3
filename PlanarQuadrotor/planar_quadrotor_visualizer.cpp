#include "planar_quadrotor_visualizer.h"
#include <SDL.h>
#include <cmath>
#include <iostream>

//zadeklarowaniej zmiennej przechowujacej kat obrotu smigla
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

    //przeskalowanie wspolrzednych
    int screen_width = 1280;
    int screen_height = 720;
    SDL_GetRendererOutputSize(gRenderer.get(), &screen_width, &screen_height);
    int q_x_screen = q_x + (screen_width/2);
    int q_y_screen = -q_y + (screen_height/2) + 29;
    //zwiekszenie stopnia obrotu smigla przy kazdym wywolaniu
    angle = angle + 60;

    //Glowna czesc drona
    SDL_Rect body = {q_x_screen - 85, q_y_screen - 50, 170, 32};
    //Pierwsze smiglo
    SDL_Rect prop1 = { q_x_screen - 95, q_y_screen - 51 , 65, 5 };
    //Drugie smiglo
    SDL_Rect prop2 = { q_x_screen + 35, q_y_screen - 51 , 65, 5 };
    //Wczytanie tekstury glownej czesci
    SDL_Surface* buffer1 = SDL_LoadBMP("body.bmp");
	if ( !buffer1 ) {
		std::cout << "Error loading image body.bmp: " << SDL_GetError() << std::endl;
    }
    SDL_Texture* body_texture;
	body_texture = SDL_CreateTextureFromSurface(gRenderer.get(),buffer1);
	SDL_FreeSurface(buffer1);
	buffer1 = NULL;
	if ( !body_texture ) {
		std::cout << "Error creating texture: " << SDL_GetError() << std::endl;
	}
    //Wczytanie tekstury dla smigiel
    SDL_Surface* buffer2 = SDL_LoadBMP("prop.bmp");
	if ( !buffer2 ) {
		std::cout << "Error loading image prop.bmp: " << SDL_GetError() << std::endl;
    }
    SDL_Texture* prop_texture;
	prop_texture = SDL_CreateTextureFromSurface(gRenderer.get(),buffer2);
	SDL_FreeSurface(buffer2);
	buffer2 = NULL;
	if ( !prop_texture ) {
		std::cout << "Error creating texture: " << SDL_GetError() << std::endl;
	}
    //Animacja drona zwiazana z katem theta
    SDL_RenderCopyEx(gRenderer.get(), body_texture, NULL, &body, q_theta * 0.3 * (180.0/M_PI), NULL, SDL_FLIP_NONE);
    //Obrot smigiel w kazdej klatce animacji
    SDL_RenderCopyEx(gRenderer.get(), prop_texture, nullptr, &prop1, angle, NULL, SDL_FLIP_NONE);
    SDL_RenderCopyEx(gRenderer.get(), prop_texture, NULL, &prop2, angle, NULL, SDL_FLIP_NONE);

    SDL_RenderPresent(gRenderer.get());
    
}
