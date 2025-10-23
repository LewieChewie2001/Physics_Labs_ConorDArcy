/*
This project uses the Raylib framework to provide us functionality for math, graphics, GUI, input etc.
See documentation here: https://www.raylib.com/, and examples here: https://www.raylib.com/examples.html
*/

#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"
#include <string>
#include <vector>

const unsigned int TARGET_FPS = 50; //frames/second
float dt = 1.0f / TARGET_FPS; //seconds/frame
float time = 0;
Vector2 birdLaunchPosition = {100, 1000};

enum FizziksShape
{
	CIRCLE,
	HALF_SPACE
};

class FizziksObjekt
{
public:
	bool isStatic = false;
	Vector2 position = { 0,0 };
	Vector2 velocity = { 0,0 };
	float mass = 1; // in kg

	std::string name = "objekt";		
	Color color = RED;

	virtual void draw() // virtual keyword is required to allow this function to be overriden
	{
		DrawCircle(position.x, position.y, 2, color);
		//DrawText(name.c_str(), position.x, position.y, 12, LIGHTGRAY);
	}

	virtual FizziksShape Shape() = 0;
	
};



class FizziksCircle : public FizziksObjekt
{
public:
	float radius; // circle radius in pixels

	void draw() override // if we want to override a parent class function, 
		// the signature (name, return type, parameter list) must match exactly
		// the override keyword makes sure you are actually overriding something. 
		// If you are not (i.e. you did it wrong) it will tell you by making a compile-time error
	{
		DrawCircle(position.x, position.y, radius, color);

		DrawText(name.c_str(), position.x, position.y, radius * 2, LIGHTGRAY);

		//Draw velocity (for fun)
		DrawLineEx(position, position + velocity, 1, color);
	}

	FizziksShape Shape() override
	{
		return CIRCLE;
	}

};

class FizziksHalfspace : public FizziksObjekt
{
private:
	float rotation = 0; 
	Vector2 normal = { 0, -1 };

public:
	void setRotationDegrees(float rotationInDegrees)
	{
		rotation = rotationInDegrees;
		normal = Vector2Rotate({ 0, -1 }, rotation * DEG2RAD);
	}

	float getRotation()
	{
		return rotation;
	}

	Vector2 getNormal()
	{
		return normal;
	}

	void draw() override
	{
		DrawCircle(position.x, position.y, 8, color);
		DrawLineEx(position, position + normal * 30, 1, color);

		Vector2 parallelToSurface = Vector2Rotate(normal, PI * 0.5f);
		DrawLineEx(position - parallelToSurface * 4000, position + parallelToSurface * 4000, 1, color);

	}

	FizziksShape Shape() override
	{
		return HALF_SPACE;
	}
};

bool CircleCircleOverlap(FizziksCircle* circleA, FizziksCircle* circleB) 
{
	Vector2 displacementFromAToB = circleB->position - circleA->position;
	float distance = Vector2Length(displacementFromAToB);//Use pythagorean theorem to get magnitude of displacement vector between circles to get a distance
	float sumOfRadii = circleA->radius + circleB->radius;
	if (sumOfRadii > distance)
	{
		return true;
	}
	else
		return false; 
}


bool CircleHalfspaceOverlap(FizziksCircle* circle, FizziksHalfspace* halfspace)
{
	
	Vector2 displacementToCircle = circle->position - halfspace->position;
	float dot = Vector2DotProduct(displacementToCircle, halfspace->getNormal());
	Vector2 vectorProjection = halfspace->getNormal() * dot;
	

	DrawLineEx(circle->position, circle->position - vectorProjection, 1, GRAY);

	Vector2 midpoint = circle->position - vectorProjection * 0.5f;
	DrawText(TextFormat("D: %6.0f", dot), midpoint.x, midpoint.y, 30, GRAY);


	if (dot < circle->radius)
	{
		return true;
	}
	else
		return false;
}


class FizziksWorld
{
private:
	unsigned int objektCount = 0;
public: 
	std::vector<FizziksObjekt*> objekts; // All objects in physics simulation
	
	Vector2 accelerationGravity = {0, 9};

	void add(FizziksObjekt* newObject) // Add to physics simulation
	{
		newObject->name = std::to_string(objektCount);
		objekts.push_back(newObject);
		objektCount++;
	}

	// Update state of all physics objects
	void update()
	{
		for (int i = 0; i < objekts.size(); i++)
		{
			objekts[i]->color = GREEN;
		}



		for (int i = 0; i < objekts.size(); i++)
		{
			FizziksObjekt* objekt = objekts[i];

			if (objekt->isStatic) continue;

			//vel = change in position / time, therefore     change in position = vel * time 
			objekt->position = objekt->position + objekt->velocity * dt;
			//accel = deltaV / time (change in velocity over time) therefore     deltaV = accel * time
			objekt->velocity = objekt->velocity + accelerationGravity * dt;
		}

		checkCollisions();
	}

	void checkCollisions()
	{
		for (int i = 0; i < objekts.size(); i++)
		{
			objekts[i]->color = GREEN;
		}
		//assuming all objects in objekts are circles...
		//for each object...
		for (int i = 0; i < objekts.size(); i++)
		{
			//check against another object...
			for (int j = i + 1; j < objekts.size(); j++)
			{
				FizziksObjekt* objektPointerA = objekts[i];
				FizziksObjekt* objektPointerB = objekts[j];

				FizziksShape shapeOfA = objektPointerA->Shape();
				FizziksShape shapeOfB = objektPointerB->Shape();

				bool didOverLap = false; 

		
				if(shapeOfA == CIRCLE && shapeOfB == CIRCLE)
				{
					didOverLap = CircleCircleOverlap((FizziksCircle*)objektPointerA, (FizziksCircle*)objektPointerB);

				}
				else if (shapeOfA == CIRCLE && shapeOfB == HALF_SPACE)
				{
					didOverLap = CircleHalfspaceOverlap((FizziksCircle*)objektPointerA, (FizziksHalfspace*)objektPointerB);
				}
				else if (shapeOfA == HALF_SPACE && shapeOfB == CIRCLE)
				{
					didOverLap = CircleHalfspaceOverlap((FizziksCircle*)objektPointerB, (FizziksHalfspace*)objektPointerA);
				}

				if (didOverLap)
				{
					objektPointerA->color = RED;
					objektPointerB->color = RED;
				}
				
			}
		}
	}
};

float speed = 100;
float angle = 0;

FizziksWorld world;
FizziksHalfspace halfspace;


//Remove objects offscreen
void cleanup()
{
	//For each object, check if it is offscreen!
	for (int i = 0; i < world.objekts.size(); i++)
	{
		FizziksObjekt* objekt = world.objekts[i];
		//Is it offscreen?
		if (	objekt->position.y > GetScreenHeight()
			||	objekt->position.y < 0
			||  objekt->position.x > GetScreenWidth()
			||  objekt->position.x < 0
			)
		{
			//Destroy!
			std::vector<FizziksObjekt*>::iterator iterator = (world.objekts.begin() + i);
			FizziksObjekt* pointerToFizziksObjekt = *iterator;
			delete pointerToFizziksObjekt;

			world.objekts.erase(iterator);
			i--;
		}
	}

}

//Changes world state
void update()
{
	dt = 1.0f / TARGET_FPS;
	time += dt;

	cleanup();
	world.update();

	if (IsKeyPressed(KEY_SPACE))
	{
		FizziksCircle* newBird = new FizziksCircle(); 
		// New keyword allocates and reserves memory on the heap
		// (as opposed to the stack, where the data will be lost on exiting scope)
		newBird->position = birdLaunchPosition;
		newBird->velocity = { speed * (float)cos(angle * DEG2RAD), -speed * (float)sin(angle * DEG2RAD) };
		
		//rand() % N produces random number from 0 to N-1
		newBird->radius = (rand() % 26) + 5; // radius from 5-30
		Color randomColor = {rand() % 256 , rand() % 256, rand() % 256, 255};
		newBird->color = randomColor;

		world.add(newBird); // Add bird to simulation
	}
}

//Display world state
void draw()
{
	BeginDrawing();
	ClearBackground(BLACK);
	DrawText("Conor D'Arcy 101446834", 10, float(GetScreenHeight() - 30), 20, LIGHTGRAY);


	GuiSliderBar(Rectangle{ 10, 15, 1000, 20 }, "", TextFormat("%.2f", time), &time, 0, 240);

	GuiSliderBar(Rectangle{ 10, 40, 500, 30 }, "Speed", TextFormat("Speed: %.0f", speed), &speed, -1000, 1000);

	GuiSliderBar(Rectangle{ 10, 80, 500, 30 }, "Angle", TextFormat("Angle: %.0f Degrees", angle), &angle, -180, 180);

	GuiSliderBar(Rectangle{ 10, 120, 500, 30 }, "Gravity Y", TextFormat("Gravity Y: %.0f Px/sec^2", world.accelerationGravity.y), &world.accelerationGravity.y, -1000, 1000);

	GuiSliderBar(Rectangle{ 10, 160, 500, 30 }, "Launch Height", TextFormat("Height: %.0f", birdLaunchPosition.y), &(birdLaunchPosition.y), 0, GetScreenHeight());
	
	DrawText(TextFormat("Obects: %i", world.objekts.size()), 10, 200, 30, LIGHTGRAY);

	DrawText(TextFormat("T: %6.2f", time), GetScreenWidth() - 140, 10, 30, LIGHTGRAY);

	Vector2 velocity = {speed * cos(angle * DEG2RAD), -speed * sin(angle * DEG2RAD)};

	DrawLineEx(birdLaunchPosition, birdLaunchPosition + velocity, 3, RED);

	GuiSliderBar(Rectangle{ 80, 200, 240, 30 }, "Halfspace X", TextFormat("%.0f", halfspace.position.x), &halfspace.position.x, 0, GetScreenWidth());
	GuiSliderBar(Rectangle{ 380, 200, 240, 30 }, "Halfspace Y", TextFormat("%.0f", halfspace.position.y), &halfspace.position.y, 0, GetScreenHeight());
	
	float halfspaceRotation = halfspace.getRotation();
	GuiSliderBar(Rectangle{ 700, 200, 200, 30 }, "Rotation", TextFormat("%.0f", halfspace.getRotation()), &halfspaceRotation, -360, 360);
	halfspace.setRotationDegrees(halfspaceRotation);


	//Draw all physics objects!
	for (int i = 0; i < world.objekts.size(); i++)
	{
		world.objekts[i]->draw();
		//Through the magic of polymorphism, we can place multiple 
		// types of objects in world.objekts. Circle, Box, Halfspace etc.
		// Then, when we call the parent function draw(), we should get the 
		// derived class behaviour specific to what that object actually is e.g.
		// Circle.draw() on a Circle, Box.draw() on a Box
	}



	EndDrawing();
}

int main()
{
	InitWindow(InitialWidth, InitialHeight, "GAME2005 Joss Moo-Young 123456789");
	SetTargetFPS(TARGET_FPS);
	halfspace.isStatic = true;
	halfspace.position = { 500, 700 };
	world.add(&halfspace);

	while (!WindowShouldClose()) // Loops TARGET_FPS times per second
	{
		update();
		draw();
	}

	CloseWindow();
	return 0;
}