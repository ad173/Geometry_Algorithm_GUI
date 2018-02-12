// AlgorithmGUI.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "GUI.h"

using namespace std;

int main(int argc, char** argv)
{
	cout << "This is the Algorithm Animation GUI\n\nFirst create a simple polygon with no intersecting lines\nNext choose the algorithm you would like to implement\n\t-X-Monotone Partitioning Algorithm (M)\n\t-Graham's Algorithm (G)\n\t-QuickHull Algorithm (Q)\n\nOnce in the Algorithm of your choice use the Space bar or Enter button to choose between a step by step or a continuous animation:\n\t-Step-by-Step (Space Bar)\n\t-Continuous (Enter)" << endl;

	HWND consoleWindow = GetConsoleWindow();

	MoveWindow(consoleWindow, 0, 500, 1366, 268, TRUE);
	//SetWindowPos(consoleWindow, 0, 0, 500, 0, 0, SWP_NOSIZE | SWP_NOZORDER);

	GUI gui;
	gui.run();
	return 0;
}

