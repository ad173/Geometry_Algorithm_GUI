#pragma once
#include <Windows.h>
#include <SDL/SDL.h>
#include <iostream>
#include <string>
#include <vector>

enum class State { RUN, EXIT };

enum Algorithm {POLYGON = 0, MONOTONE = 1, GRAHAM = 2, QUICKHULL = 3 };

struct PolygonPoint
{
	POINT p;
	PolygonPoint *next;
	PolygonPoint *prev;
};

class GUI
{
public:
	GUI();
	~GUI();

	void run();
	void processInput();
	bool continuous;
	bool nextStep;
	int getWidth() { return _screenWidth; }
	int getHeight() { return _screenHeight; }

private:
	void initSystems();
	void Loop();
	void createPoint(POINT p);
	void RunAlgorithm(Algorithm);

	Algorithm currentAlgo;
	SDL_Window* _window;
	SDL_Renderer* _renderer;
	int _screenWidth;
	int _screenHeight;
	int mouse_X;
	int mouse_Y;
	bool finished;
	bool stop;
	std::vector<POINT> *vertexList;
	PolygonPoint *poly;
	POINT p;
	State _state;
};

