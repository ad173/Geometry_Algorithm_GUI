#include "GUI.h"
#include <algorithm>

const double pi = 3.1415926535897;

using namespace std;

void fatalError(std::string errorString)
{
	std::cout << errorString << std::endl;
	std::cout << "Enter any key to quit...";
	int tmp;
	std::cin >> tmp;
	exit(1);
}

GUI::GUI()
{
	_window = nullptr;
	_renderer = nullptr;
	_screenHeight = 460;
	_screenWidth = 1366;
	mouse_X = 0;
	mouse_Y = 0;
	p.x = 0;
	p.y = 0;
	poly = nullptr;
	_state = State::RUN;
	currentAlgo = Algorithm(POLYGON);
	vertexList = new std::vector<POINT>;
	finished = false;
	stop = false;
}

GUI::~GUI()
{
}

void GUI::run()
{
	initSystems();
	Loop();
}

void GUI::initSystems()
{
	SDL_Init(SDL_INIT_EVERYTHING);

	_window = SDL_CreateWindow("Convex Hull - Incremental Algorithm", 0, 30, _screenWidth, _screenHeight, SDL_WINDOW_OPENGL);
	if (_window == nullptr)
	{
		fatalError("SDL Window could not be created!");
	}

	_renderer = SDL_CreateRenderer(_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
	if (_renderer == nullptr)
	{
		fatalError("SDL Renderer could not be created!");
	}

	SDL_GLContext context = SDL_GL_CreateContext(_window);
	if (context == nullptr)
	{
		fatalError("SDL_GL context could not be created!");
	}

	SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
	SDL_RenderClear(_renderer);
	SDL_RenderPresent(_renderer);
}

void GUI::Loop()
{
	while (_state != State::EXIT)
	{
		processInput();
	}
}

void GUI::processInput()
{
	SDL_Event e;
	while (SDL_PollEvent(&e) == 1)
	{
		switch (e.type)
		{
		case SDL_QUIT:
			_state = State::EXIT;
			break;
		case SDL_MOUSEMOTION:
			mouse_X = e.motion.x;
			mouse_Y = e.motion.y;
			break;
		case SDL_MOUSEBUTTONDOWN:
			if (currentAlgo == Algorithm(POLYGON))
			{
				p.x = mouse_X;
				p.y = mouse_Y;
			}
			break;
		case SDL_MOUSEBUTTONUP:
			if (currentAlgo == Algorithm(POLYGON) && !finished)
			{
				createPoint(p);
			}
			else if (currentAlgo == Algorithm(POLYGON) && finished)
			{
				PolygonPoint *t = poly;
				while (t != nullptr && t->next != poly)
				{
					PolygonPoint *temp = t->next;
					delete t;
					t = temp;
				}
				delete t;
				delete vertexList;
				vertexList = new std::vector<POINT>;
				poly = nullptr;
				SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
				SDL_RenderClear(_renderer);

				SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
				SDL_RenderPresent(_renderer);

				finished = false;
			}
			break;
		case SDL_KEYDOWN:
			switch (e.key.keysym.sym)
			{
			case SDLK_LSHIFT:
			case SDLK_RSHIFT:
				if (currentAlgo != Algorithm(POLYGON))
				{
					stop = !stop;
					if (!stop)
					{
						return;
					}
					while (stop)
					{
						processInput();
					}
				}
			case SDLK_SPACE:
				if (currentAlgo != Algorithm(POLYGON))
				{
					nextStep = true;
				}
				break;
			case SDLK_RETURN:
				if (currentAlgo != Algorithm(POLYGON))
				{
					continuous = true;
				}
				break;
			case SDLK_m:
				if (currentAlgo == Algorithm(POLYGON))
				{
					currentAlgo = Algorithm(MONOTONE);
					RunAlgorithm(currentAlgo);
					currentAlgo = Algorithm(POLYGON);
					finished = true;
					continuous = false;
				}
				break;
			case SDLK_g:
				if (currentAlgo == Algorithm(POLYGON))
				{
					currentAlgo = Algorithm(GRAHAM);
					RunAlgorithm(currentAlgo);
					currentAlgo = Algorithm(POLYGON);
					finished = true;
					continuous = false;
				}
				break;
			case SDLK_q:
				if (currentAlgo == Algorithm(POLYGON))
				{
					currentAlgo = Algorithm(QUICKHULL);
					RunAlgorithm(currentAlgo);
					currentAlgo = Algorithm(POLYGON);
					finished = true;
					continuous = false;
				}
				break;
			}
		}
	}
}

int AreaSign(POINT a, POINT b, POINT c)
{
	return ((b.x - a.x)*(c.y - a.y) - (c.x - a.x)*(b.y - a.y));
}

bool Left(POINT p, POINT v1, POINT v2)
{
	bool results = AreaSign(v1, v2, p) > 0;
	return results;
}

bool Right(POINT p, POINT v1, POINT v2)
{
	return AreaSign(v1, v2, p) < 0;
}

bool Collinear(POINT p, POINT v1, POINT v2)
{
	return AreaSign(v1, v2, p) == 0;
}

bool Intersect(POINT a, POINT b, POINT c, POINT d)
{
	if ((Left(a, b, c) != Left(a, b, d)) && (Left(c, d, a) != Left(c, d, b)))
	{
		return true;
	}
	return false;
}

bool Between(POINT a, POINT b, POINT c)
{
	if (!Collinear(a, b, c))
	{
		return false;
	}
	if (a.x != b.x)
	{
		return ((a.x <= c.x) && (c.x <= b.x)) || ((a.x >= c.x) && (c.x >= b.x));
	}
	else
	{
		return ((a.y <= c.y) && (c.y <= b.y)) || ((a.y >= c.y) && (c.y >= b.y));
	}
}

void removeOthers(PolygonPoint *start, PolygonPoint *original, PolygonPoint *end)
{
	PolygonPoint *temp = start->next;
	while (temp != end)
	{
		if (temp == original)
		{
			original = end;
		}
		temp = temp->next;
		delete temp->prev;
	}
}

bool Simple(PolygonPoint *p)
{
	bool first = true;
	if (p != nullptr && p->next != nullptr)
	{
		PolygonPoint *t = p;
		PolygonPoint *temp;
		do
		{
			temp = t->next->next;
			while (temp != t->prev && temp != t)
			{
				if (Intersect(t->p, t->next->p, temp->p, temp->next->p))
				{
					return false;
				}
				temp = temp->next;
			}
			t = t->next;
		} while (t != p);
	}
	else
	{
		return false;
	}
	return true;
}

//Begining of Quickhull Algorithm
double distance(POINT p, POINT q, POINT s, SDL_Renderer* r) {
	double pq_slope = (double)(p.y - q.y) / (double)(p.x - q.x);
	double pq_B = (double)p.y - (double)p.x*pq_slope;

	double s_slope = -(1/pq_slope);
	double s_B = (double)s.y - (double)s.x*s_slope;

	double x = (s_B - pq_B) / (pq_slope - s_slope);
	double y = x * s_slope + s_B;
	SDL_SetRenderDrawColor(r, 255, 0, 0, 255);

	SDL_RenderDrawLine(r, x, y, s.x, s.y);
	SDL_RenderPresent(r);

	double x2 = pow((double)s.x - x, 2);
	double y2 = pow((double)s.y - y, 2);

	return sqrt(x2 + y2);
}

void find_hull(bool left, bool top, POINT p, POINT q, vector<POINT> &set, vector<POINT> *CH, GUI* g, vector<POINT>* points, SDL_Renderer* r)
{
	// Return if set is empty
	if (set.empty()) {
		return;
	}
	SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
	SDL_RenderClear(r);

	SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

	for (int i = 0; i < points->size(); i++)
	{
		SDL_RenderDrawPoint(r, points->at(i).x, points->at(i).y);
	}

	while (!g->continuous && !g->nextStep)
	{
		g->processInput();
	}

	// Find insertion point in convex hull
	vector<POINT>::iterator insertion_iter;
	for (int i = 0; i < CH->size(); i++)
	{
		if (top && CH->at(i).x == q.x && CH->at(i).y == q.y)
		{
			insertion_iter = CH->begin() + i;
			break;
		}
		else if(!top && CH->at(i).x == p.x && CH->at(i).y == p.y)
		{
			insertion_iter = CH->begin() + i;
			break;
		}
	}

	auto max_p{ set.begin() };
	auto max_dist{ INT_MIN };

	// Find point on side of line with max distance from line
	for (auto it{ set.begin() }; it != set.end(); ++it) {
		auto dist{ distance(p, q, *it, r) };
 		max_p = dist > max_dist ? it : max_p;
		max_dist = dist > max_dist ? dist : max_dist;
	}

	// Insert point into convex hull and remove from set
	POINT s = *max_p;
	CH->insert(insertion_iter, s);
	set.erase(max_p);

	std::vector<POINT> pleft;
	std::vector<POINT> qleft;

	// Determine points to the left of line formed by point p and s and line formed by q and s
	for (int i = 0; i < set.size(); i++) {
		if (top)
		{
			if (left)
			{
				if (Left(set.at(i), q, s))
				{
					qleft.push_back(set.at(i));
				}
				if (Left(set.at(i), s, p))
				{
					pleft.push_back(set.at(i));
				}
			}
			else
			{
				if (Left(set.at(i), s, p))
				{
					pleft.push_back(set.at(i));
				}
				if (Left(set.at(i), q, s))
				{
					qleft.push_back(set.at(i));
				}
			}
		}
		else
		{
			if (left)
			{
				if (Right(set.at(i), s, p))
				{
					pleft.push_back(set.at(i));
				}
				if (Right(set.at(i), q, s))
				{
					qleft.push_back(set.at(i));
				}
			}
			else
			{
				if (Right(set.at(i), s, p))
				{
					pleft.push_back(set.at(i));
				}
				if (Right(set.at(i), q, s))
				{
					qleft.push_back(set.at(i));
				}
			}
		}
	}

	for (int i = 0; i < CH->size() - 1; i++)
	{
		SDL_RenderDrawLine(r, CH->at(i).x, CH->at(i).y, CH->at(i + 1).x, CH->at(i + 1).y);
	}
	SDL_RenderDrawLine(r, CH->at(CH->size() - 1).x, CH->at(CH->size() - 1).y, CH->at(0).x, CH->at(0).y);

	SDL_RenderPresent(r);
	g->nextStep = false;

	// Repeat recursively
	find_hull(left, top, p, s, pleft, CH, g, points, r);
	find_hull(!left, top, s, q, qleft, CH, g, points, r);
}

void QuickHull(vector<POINT>* p, SDL_Renderer* r, GUI* g)
{
	SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
	SDL_RenderClear(r);

	SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

	for (int i = 0; i < p->size(); i++)
	{
		SDL_RenderDrawPoint(r, p->at(i).x, p->at(i).y);
	}
	SDL_RenderPresent(r);

	while (!g->continuous && !g->nextStep)
	{
		g->processInput();
	}

	vector<POINT>* CHull = new vector<POINT>;
	if (p->size() < 3) {
		std::copy(p->begin(), p->end(), CHull->begin());
		return;
	}

	auto min_iter{ p->begin() }, max_iter{ p->begin() };
	auto min_x{ INT_MAX }, max_x{ INT_MIN };

	// Find points with minimum and maximum x coordinates - always part of convex hull
	for (auto it{ p->begin() }; it != p->end(); ++it) {

		min_iter = it->x < min_x ? it : min_iter;
		min_x = it->x < min_x ? it->x : min_x;

		max_iter = it->x > max_x ? it : max_iter;
		max_x = it->x > max_x ? it->x : max_x;

	}

	// Add the min/max points to convex hull and remove them from 'points' set
	CHull->push_back(*min_iter);
	CHull->push_back(*max_iter);
	POINT temp1 = *min_iter;
	POINT temp2 = *max_iter;

	SDL_RenderDrawLine(r, temp1.x, temp1.y, temp2.x, temp2.y);

	SDL_RenderPresent(r);

	vector<POINT> left;
	vector<POINT> right;

	// Use line formed by p & q to build two subsets of points, left / right points
	for (int i = 0; i < p->size(); i++) {
		if ((p->at(i).x == temp1.x && p->at(i).y == temp1.y) || (p->at(i).x == temp2.x && p->at(i).y == temp2.y))
		{
			continue;
		}
		if (Left(p->at(i), temp1, temp2)) {
			left.push_back(p->at(i));
		}
		else if (Right(p->at(i), temp1, temp2))
		{
			right.push_back(p->at(i));
		}
	}

	g->nextStep = false;

	// Call find hull with each set of points
	find_hull(true, true, temp1, temp2, right, CHull, g, p, r);
	find_hull(false, false, temp1, temp2, left, CHull, g, p, r);

	SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
	SDL_RenderClear(r);

	SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

	for (int i = 0; i < p->size(); i++)
	{
		SDL_RenderDrawPoint(r, p->at(i).x, p->at(i).y);
	}

	for (int i = 0; i < CHull->size() - 1; i++)
	{
		SDL_RenderDrawLine(r, CHull->at(i).x, CHull->at(i).y, CHull->at(i + 1).x, CHull->at(i + 1).y);
	}
	SDL_RenderDrawLine(r, CHull->at(CHull->size() - 1).x, CHull->at(CHull->size() - 1).y, CHull->at(0).x, CHull->at(0).y);

	SDL_RenderPresent(r);
}

//Begining of Graham algorithm
void SortX(vector<POINT>* p)
{
	int index;
	double minAngle;
	for (int i = 0; i < p->size(); i++)
	{
		index = i;
		minAngle = 400;
		for (int j = i; j < p->size(); j++)
		{
			if (i == 0)
			{
				if (p->at(j).y > p->at(index).y)
				{
					index = j;
				}
			}
			else
			{
				double xDiff = p->at(0).x - p->at(j).x;
				double yDiff = p->at(0).y - p->at(j).y;
				if (xDiff < 0)
				{
					double TempAngle = atan2(yDiff, -xDiff) * 180 / pi;
					if (TempAngle < minAngle)
					{
						minAngle = TempAngle;
						index = j;
					}
				}
				else
				{
					double TempAngle = (pi - atan2(yDiff, xDiff)) * 180 / pi;
					if (TempAngle < minAngle)
					{
						minAngle = TempAngle;
						index = j;
					}
				}
			}
		}

		if (index != i)
		{
			POINT temp = p->at(i);
			p->at(i) = p->at(index);
			p->at(index) = temp;
		}
	}
}

void Graham(vector<POINT>* p, SDL_Renderer* r, GUI* g)
{
	SortX(p);
	vector<POINT> CHull;
	int index = 0;

	SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
	SDL_RenderClear(r);

	SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

	for (int i = 0; i < p->size(); i++)
	{
		SDL_RenderDrawPoint(r, p->at(i).x, p->at(i).y);
	}
	SDL_RenderPresent(r);

	while (index != p->size())
	{
		if (g->continuous || g->nextStep)
		{
  			if (CHull.size() < 3)
			{
				if (CHull.size() >= 2 && Collinear(p->at(index), CHull.at(CHull.size() - 1), CHull.at(CHull.size() - 2)))
				{
					CHull.at(CHull.size() - 1) = p->at(index);
				}
				else
				{
					CHull.push_back(p->at(index));
				}
				index++;
			}
			else
			{
  				if (Left(p->at(index), CHull.at(CHull.size() - 1), CHull.at(CHull.size() - 2)))
				{
					CHull.push_back(p->at(index));
				}
				else if(Collinear(p->at(index), CHull.at(CHull.size() - 1), CHull.at(CHull.size() - 2)))
				{
					CHull.at(CHull.size() - 1) = p->at(index);
				}
				else
				{
					while (!Left(p->at(index), CHull.at(CHull.size() - 1), CHull.at(CHull.size() - 2)))
					{
						CHull.pop_back();
					}
					CHull.push_back(p->at(index));
				}
				index++;
			}

			if (g->nextStep)
			{
				g->nextStep = false;
			}
			SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
			SDL_RenderClear(r);

			SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

			for (int i = 0; i < p->size(); i++)
			{
				SDL_RenderDrawPoint(r, p->at(i).x, p->at(i).y);
			}

			for (int i = 0; i < CHull.size() - 1; i++)
			{
				SDL_RenderDrawLine(r, CHull.at(i).x, CHull.at(i).y, CHull.at(i + 1).x, CHull.at(i + 1).y);
			}
			SDL_RenderDrawLine(r, CHull.at(CHull.size() - 1).x, CHull.at(CHull.size() - 1).y, CHull.at(0).x, CHull.at(0).y);
			SDL_RenderPresent(r);
		}
		else
		{
			g->processInput();
		}
	}
	return;
}

//Begining of Monotone Algorithm
pair<POINT, POINT> CreateMonoEdge(int i, vector<POINT>* YSorted, PolygonPoint* t, bool pos)
{
	int index = i;
	PolygonPoint* temp = t->next;
	bool flag = false;
	if (index == YSorted->size())
	{
		pair<POINT, POINT> p = { { -1,-1 },{ -1,-1 } };
		return p;
	}
	if (pos)
	{
		index--;
	}
	else
	{
		index++;
	}

	while (index >= 0)
	{
		temp = temp = t->next;
		flag = false;
		while (temp != t && temp->next != t)
		{
			if ((temp->p.x == YSorted->at(index).x && temp->p.y == YSorted->at(index).y) || (temp->next->p.x == YSorted->at(index).x && temp->next->p.y == YSorted->at(index).y))
			{
				temp = temp->next;
			}
			else
			{
				if (Intersect(t->p, YSorted->at(index), temp->p, temp->next->p))
				{
					flag = true;
				}
				if (flag)
				{
					if (pos)
					{
						index--;
					}
					else
					{
						index++;
					}
					break;
				}
				else
				{
					temp = temp->next;
				}
			}
		}
		if (!flag)
		{
			break;
		}
	}
	if (pos)
	{
		if (index >= 0)
		{
			pair<POINT, POINT> MonoPoint = { t->p, YSorted->at(index) };
			return MonoPoint;
		}
	}
	else
	{
		if (index < YSorted->size())
		{
			pair<POINT, POINT> MonoPoint = { t->p, YSorted->at(index) };
			return MonoPoint;
		}
	}
	pair<POINT, POINT> p = { {-1,-1}, { -1,-1 } };
	return p;
}

void Monotone(PolygonPoint* poly, vector<POINT>* p, SDL_Renderer* r, GUI* g)
{
	vector<pair<POINT, POINT>>* XMono = new vector<pair<POINT, POINT>>;
	vector<POINT>* YSorted = new vector<POINT>;
	for (int i = 0; i < p->size(); i++)
	{
		YSorted->push_back(p->at(i));
	}

	for (int i = 0; i < YSorted->size() - 1; i++)
	{
		int temp = i;
		for (int j = i + 1; j < YSorted->size(); j++)
		{
			if (YSorted->at(j).y < YSorted->at(temp).y)
			{
				temp = j;
			}
		}
		if (temp != i)
		{
			POINT t = YSorted->at(i);
			YSorted->at(i) = YSorted->at(temp);
			YSorted->at(temp) = t;
		}
	}

	int CorCC = 0; // Positive = CounterClockwise
				   //Negative = Clockwise

	for (int i = 0; i < p->size(); i++)
	{
		if (i == p->size() - 1)
		{
			CorCC += (p->at(0).x - p->at(i).x)*(p->at(0).y + p->at(i).y);
		}
		else
		{
			CorCC += (p->at(i + 1).x - p->at(i).x)*(p->at(i + 1).y + p->at(i).y);
		}
	}
	SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
	SDL_RenderClear(r);

	SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

	for (int i = 0; i < p->size(); i++)
	{
		SDL_RenderDrawPoint(r, p->at(i).x, p->at(i).y);
	}

	PolygonPoint* t = poly;
	while (t->next != nullptr && t->next != poly)
	{
		SDL_RenderDrawLine(r, t->p.x, t->p.y, t->next->p.x, t->next->p.y);
		t = t->next;
	}

	SDL_RenderDrawLine(r, t->p.x, t->p.y, t->next->p.x, t->next->p.y);

	for (int i = 0; i < YSorted->size(); i++)
	{
		while (!g->continuous && !g->nextStep)
		{
 			g->processInput();
		}
		g->nextStep = false;

		SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
		SDL_RenderClear(r);

		SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

		for (int j = 0; j < p->size(); j++)
		{
			SDL_RenderDrawPoint(r, p->at(j).x, p->at(j).y);
		}

		PolygonPoint* t = poly;
		while (t->next != nullptr && t->next != poly)
		{
			SDL_RenderDrawLine(r, t->p.x, t->p.y, t->next->p.x, t->next->p.y);
			t = t->next;
		}

		SDL_RenderDrawLine(r, t->p.x, t->p.y, t->next->p.x, t->next->p.y);
		t = t->next;
		for (int j = 0; j < XMono->size(); j++)
		{
			SDL_RenderDrawLine(r, XMono->at(j).first.x, XMono->at(j).first.y, XMono->at(j).second.x, XMono->at(j).second.y);
		}

		t = poly;
		while (t->p.x != YSorted->at(i).x || t->p.y != YSorted->at(i).y)
		{
			t = t->next;
		}

		SDL_SetRenderDrawColor(r, 255, 0, 0, 255);

		SDL_RenderDrawLine(r, 0, t->p.y, g->getWidth() - 1, t->p.y);
		SDL_RenderPresent(r);

		SDL_SetRenderDrawColor(r, 0, 0, 0, 255);

		POINT YTest = { (((t->p.x + t->next->p.x) / 2) + ((t->p.x + t->prev->p.x) / 2)) / 2 ,
						(((t->p.y + t->next->p.y) / 2) + ((t->p.y + t->prev->p.y) / 2)) / 2 };

		if (CorCC < 0)
		{
			if (Right(YTest, t->p, t->next->p) && Left(YTest, t->p, t->prev->p))
			{
				if (t->p.y <= t->next->p.y && t->p.y <= t->prev->p.y)
				{
					pair<POINT, POINT> MonoPoint = CreateMonoEdge(i, YSorted, t, true);
					if (MonoPoint.first.x != -1 && MonoPoint.first.y != -1)
					{
						XMono->push_back(MonoPoint);
					}
				}
				else if (t->p.y >= t->next->p.y && t->p.y >= t->prev->p.y)
				{
					pair<POINT, POINT> MonoPoint = CreateMonoEdge(i, YSorted, t, false);
					if (MonoPoint.first.x != -1 && MonoPoint.first.y != -1)
					{
						XMono->push_back(MonoPoint);
					}
				}
			}
		}
		else
		{
			if (Left(YTest, t->p, t->next->p) && Right(YTest, t->p, t->prev->p))
			{
				if (t->p.y <= t->next->p.y && t->p.y <= t->prev->p.y)
				{
					pair<POINT, POINT> MonoPoint = CreateMonoEdge(i, YSorted, t, true);
					if (MonoPoint.first.x != -1 && MonoPoint.first.y != -1)
					{
						XMono->push_back(MonoPoint);
					}
				}
				else if (t->p.y >= t->next->p.y && t->p.y >= t->prev->p.y)
				{
					pair<POINT, POINT> MonoPoint = CreateMonoEdge(i, YSorted, t, false);
					if (MonoPoint.first.x != -1 && MonoPoint.first.y != -1)
					{
						XMono->push_back(MonoPoint);
					}
				}
			}
		}
	}
}

void GUI::RunAlgorithm(Algorithm a)
{
	if (!Simple(poly))
	{
		currentAlgo = Algorithm(POLYGON);
		cout << "Make Polygon a Simple Polygon!" << endl;
		PolygonPoint *t = poly;
		while (t->next != poly)
		{
			PolygonPoint *temp = t->next;
			delete t;
			t = temp;
		}
		delete t;
		delete vertexList;
		vertexList = new std::vector<POINT>;
		poly = nullptr;
		SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
		SDL_RenderClear(_renderer);

		SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);
		SDL_RenderPresent(_renderer);
		return;
	}

	SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
	SDL_RenderClear(_renderer);

	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);

	for (int i = 0; i < vertexList->size(); i++)
	{
		SDL_RenderDrawPoint(_renderer, vertexList->at(i).x, vertexList->at(i).y);
	}

	PolygonPoint *t = poly;

	if (a == Algorithm(QUICKHULL))
	{
		QuickHull(vertexList, _renderer, this);
	}
	else if (a == Algorithm(GRAHAM))
	{
		Graham(vertexList, _renderer, this);
	}
	else if (a == Algorithm(MONOTONE))
	{
		Monotone(poly, vertexList, _renderer, this);
	}

	SDL_RenderPresent(_renderer);
}

void GUI::createPoint(POINT p)
{
	vertexList->push_back(p);
	if (poly == nullptr)
	{
		poly = new PolygonPoint;
		poly->p = p;
		poly->next = nullptr;
		poly->prev = nullptr;
	}
	else if(poly->prev == nullptr)
	{
		poly->next = new PolygonPoint;
		poly->next->p = p;
		poly->next->next = poly;
		poly->next->prev = poly;
		poly->prev = poly->next;
	}
	else
	{
		PolygonPoint *temp = new PolygonPoint;
		temp->p = p;
		if (Collinear(temp->p, poly->p, poly->prev->p))
		{
			delete temp;
		}
		else
		{
			temp->next = poly;
			temp->prev = poly->prev;
			poly->prev = temp;
			temp->prev->next = temp;
		}
	}

	SDL_SetRenderDrawColor(_renderer, 255, 255, 255, 255);
	SDL_RenderClear(_renderer);

	SDL_SetRenderDrawColor(_renderer, 0, 0, 0, 255);

	for (int i = 0; i < vertexList->size(); i++)
	{
		SDL_RenderDrawPoint(_renderer, vertexList->at(i).x, vertexList->at(i).y);
	}

	PolygonPoint *t = poly;
	while (currentAlgo == Algorithm(POLYGON) && t->next != nullptr && t->next != poly)
	{
		SDL_RenderDrawLine(_renderer, t->p.x, t->p.y, t->next->p.x, t->next->p.y);
		t = t->next;
	}
	if (currentAlgo == Algorithm(POLYGON) && t->next != nullptr)
	{
		SDL_RenderDrawLine(_renderer, t->p.x, t->p.y, t->next->p.x, t->next->p.y);
	}
	SDL_RenderPresent(_renderer);
}