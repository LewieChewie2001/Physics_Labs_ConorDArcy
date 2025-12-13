#include <iostream>
#include "raylib.h"
#include "raymath.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#include "game.h"
#include <cmath>
#include<vector>
#include <string>

const unsigned int TARGET_FPS = 50;

float angle = 45.0f;
float speed = 0.0f;
float dt = 1.0f / TARGET_FPS;
float stime = 0;
const int OFFSCREEN = 300;
float coefficientOfFriction = 0.5f;
float sphereMass = 5.0f;
float restitutionCoefficient = 0.9f; 
bool dragging = false;
Vector2 dragStart = { 0,0 };
Vector2 dragEnd = { 0,0 };
float maxPower = 800.0f;
int currentBirdType = 0;

using namespace std;
float ClampF(float v, float min, float max) 
{
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

enum ObjectShape 
{

    CIRCLE,
    HALF_SPACE,
    AABB
};
class Object 
{
    friend class PhysicsSimulation;
public:
    bool isStatic = false;
    Vector2 position;
    Vector2 velocity;
    float mass = 1;
    string name = "object";
    Vector2 netForce = { 0,0 };
    Color colour;
    Object() 
    {
        position = { 0,0 };
        velocity = { 0,0 };
    }
    virtual void draw() 
    {
        DrawCircle(position.x, position.y, 12, colour);
        DrawLineEx(position, position + velocity, 2, colour);
    }
    virtual ObjectShape Shape() = 0;
};
class ObjectBox : public Object 
{
public:
    Vector2 size;

};
class ObjectCircle : public Object 
{
public:
    float radius;
	float grippiness = 0.1f;
	float bounciness = 0.9f;  
    bool isPig = false;
    float toughness = 300.0f;
    bool toRemove = false;

    void draw() override 
    {
        DrawCircle(position.x, position.y, radius, colour);
        DrawLineEx(position, position + velocity, 2, colour);
    }
    ObjectShape Shape() override 
    {
        return CIRCLE;
    }
};
class ObjectAABB : public Object 
{
public:
    Vector2 halfSize; 
    bool isBullet = false;

    ObjectAABB(Vector2 pos, Vector2 size, bool isStaticObj, Color c, float m) 
    {
        position = pos;
        halfSize = size / 2.0f;
        isStatic = isStaticObj;
        colour = c;
        mass = m;
    }

    ObjectShape Shape() override 
    {
        return AABB;
    }

    void draw() override 
    {
        DrawRectangle(position.x - halfSize.x, position.y - halfSize.y, halfSize.x * 2, halfSize.y * 2, colour);
        DrawLineEx(position, position + velocity, 2, colour);
    }
    Vector2 Min() { return position - halfSize; }
    Vector2 Max() { return position + halfSize; }
};

class ObjectHalfSpace : public Object 
{
private:
    float rotation = 0;
	
    Vector2 normal = { 0,-1 };

public:
    float grippiness = 0.1f;
    float bounciness = 0.9f;
    void setRotationDegree(float rotationInDegree) 
    {
        rotation = rotationInDegree;
        normal = Vector2Rotate({ 0,-1 }, rotation * DEG2RAD);
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
        DrawCircle(position.x, position.y, 8, colour);
        DrawLineEx(position, position + normal * 30, 1, colour);
        Vector2 parallelToSurface = Vector2Rotate(normal, PI * 0.5);
        DrawLineEx(position - parallelToSurface * 4000, position + parallelToSurface * 4000, 1, colour);

    }
    ObjectShape Shape() override 
    {
        return HALF_SPACE;
    }
};
bool CircleOverlap(ObjectCircle* A, ObjectCircle* B) 
{
    Vector2 displacement = B->position - A->position;
    float distance = Vector2Length(displacement);
    float sumR = A->radius + B->radius;
    if (sumR >= distance) return true;
    else return false;
}

bool CircleCollisionResponse(ObjectCircle* A, ObjectCircle* B) 
{
    Vector2 displacement = B->position - A->position;
    float distance = Vector2Length(displacement);
    float sumR = A->radius + B->radius;
    float overlap = sumR - distance;
    if (overlap > 0) 
    {
		Vector2 normal = displacement / distance;
        A->position -= displacement / distance * overlap * 0.5f;
        B->position += displacement / distance * overlap * 0.5f;
		Vector2 velocityBRelativeA = B->velocity - A->velocity;
		float closingVelocity1D = Vector2DotProduct(velocityBRelativeA, normal );
        if (closingVelocity1D >= 0) return true;

        float restitution = A->bounciness * B->bounciness;
		float totalMass = A->mass + B->mass;   
		float impulseMagnitude = (1.0f + restitution) * closingVelocity1D * (A->mass * B->mass) / totalMass;
		Vector2 impulseB = normal * -impulseMagnitude;
        Vector2 impulseA = normal * impulseMagnitude;
		A->velocity += impulseA / A->mass;
		B->velocity += impulseB / B->mass;

        return true;
    }
    else 
    {
        return false;
    }
}



bool CircleHalfSpaceOverlap(ObjectCircle* circle, ObjectHalfSpace* halfSpace) 
{
    Vector2 displacement = circle->position - halfSpace->position;
    float dot = Vector2DotProduct(displacement, halfSpace->getNormal());
    Vector2 vectorProjection = halfSpace->getNormal() * dot;
    DrawLineEx(circle->position, circle->position - vectorProjection, 1, GRAY);
    Vector2 midpoint = circle->position - vectorProjection * 0.5f;
    DrawText(TextFormat("D: %6.0f", dot), midpoint.x, midpoint.y, 30, LIGHTGRAY);
    return dot < circle->radius;
}
bool AABBCollisionResponse(ObjectAABB* A, ObjectAABB* B)
{
    float dx = B->position.x - A->position.x;
    float overlapX = (A->halfSize.x + B->halfSize.x) - fabsf(dx);
    if (overlapX <= 0) return false;
    float dy = B->position.y - A->position.y;
    float overlapY = (A->halfSize.y + B->halfSize.y) - fabsf(dy);
    if (overlapY <= 0) return false;
    float totalMass = A->mass + B->mass;
    Vector2 normal;
    float overlap;

    if (overlapX < overlapY)
    {
        overlap = overlapX;
        if (dx < 0)
        {
            normal = { -1.0f, 0.0f };
        }
        else
        {
            normal = { 1.0f, 0.0f };
        }

    }
    else
    {
        overlap = overlapY;
        if (dy < 0)
        {
            normal = { 0.0f, -1.0f };
        }
        else
        {
            normal = { 0.0f, 1.0f };
        }

    }

    if (!A->isStatic) A->position -= normal * (overlap * (B->mass / totalMass));
    if (!B->isStatic) B->position += normal * (overlap * (A->mass / totalMass));

    Vector2 relVel = B->velocity - A->velocity;
    float closingVel = Vector2DotProduct(relVel, normal);

    if (closingVel < 0)
    {
        float restitution = 0.6f;
        float j = -(1 + restitution) * closingVel * (A->mass * B->mass) / totalMass;

        if (!A->isStatic) A->velocity -= normal * (j / A->mass);
        if (!B->isStatic) B->velocity += normal * (j / B->mass);
    }

    return true;
}







bool CircleAABBCollisionResponse(ObjectCircle* circle, ObjectAABB* box)
{

    Vector2 closestPoint;
    closestPoint.x = ClampF(circle->position.x, box->Min().x, box->Max().x);
    closestPoint.y = ClampF(circle->position.y, box->Min().y, box->Max().y);

    Vector2 disp = circle->position - closestPoint;
    float dist = Vector2Length(disp);

    if (dist >= circle->radius) return false;

    float overlap = circle->radius - dist;
    Vector2 normal;

    if (dist == 0)
    {
        normal = { 0.0f, -1.0f };
    }
    else
    {
        normal = disp / dist;
    }

    float restitution = circle->bounciness * 0.6f; 

    float totalMass = circle->mass + box->mass;


    if (!circle->isStatic)
        circle->position += normal * overlap * (box->mass / totalMass);
    if (!box->isStatic)
        box->position -= normal * overlap * (circle->mass / totalMass);

    float closingVelocity = Vector2DotProduct(circle->velocity - box->velocity, normal);

    if (closingVelocity < 0)
    {
        float impulse = -(1.0f + restitution) * closingVelocity;
        impulse *= (circle->mass * box->mass) / totalMass;

        if (!circle->isStatic)
            circle->velocity += normal * (impulse / circle->mass);
        if (!box->isStatic)
            box->velocity -= normal * (impulse / box->mass);
    }

    return true;
}


bool CircleHalfSpaceCollisionResponse(ObjectCircle* circle, ObjectHalfSpace* halfSpace);
float GetMomentum(Object* obj) 
{
    return obj->mass * Vector2Length(obj->velocity);
}

class PhysicsSimulation 
{
public:
    std::vector<Object*> objekts;
    Vector2 gravityAcceleration = { 0,200 };
    void update() 
    {
        resetNetForces();
        addGravityForce();
        checkCollisions();
        applyKinematics();
    }
    void add(Object* newObj) 
    {
        newObj->name = to_string(objekts.size());
        objekts.push_back(newObj);
    }
    void resetNetForces() 
    {
        for (int i = 0;i < objekts.size();i++) 
        {
            objekts[i]->netForce = { 0,0 };
        }
    }
    void addGravityForce() 
    {
        for (int i = 0;i < objekts.size();i++) 
        {
            if (objekts[i]->isStatic) continue;
            Vector2 ForceGravity = gravityAcceleration * objekts[i]->mass;
            objekts[i]->netForce += ForceGravity;
        }
    }
    void applyKinematics() 
    {
        for (int i = 0;i < objekts.size();i++) 
        {
            if (objekts[i]->isStatic) continue;
            objekts[i]->position += objekts[i]->velocity * dt;
            objekts[i]->velocity += (objekts[i]->netForce / objekts[i]->mass) * dt;
        }
    }

    void checkCollisions() 
    {
        
        for (int i = 0;i < objekts.size();i++) 
        {
            for (int j = i + 1;j < objekts.size();j++) 
            {
                Object* PointerA = objekts[i];
                Object* PointerB = objekts[j];

                ObjectShape shapeofA = PointerA->Shape();
                ObjectShape shapeofB = PointerB->Shape();
                bool didOverlap = false;
                if (shapeofA == CIRCLE && shapeofB == CIRCLE) 
                {
                    didOverlap = CircleCollisionResponse((ObjectCircle*)PointerA, (ObjectCircle*)PointerB);
                }
                else if (shapeofA == CIRCLE && shapeofB == HALF_SPACE) 
                {
                    didOverlap = CircleHalfSpaceCollisionResponse((ObjectCircle*)PointerA, (ObjectHalfSpace*)PointerB);
                }
                else if (shapeofA == HALF_SPACE && shapeofB == CIRCLE) 
                {
                    didOverlap = CircleHalfSpaceCollisionResponse((ObjectCircle*)PointerB, (ObjectHalfSpace*)PointerA);
                }
                else if (shapeofA == AABB && shapeofB == AABB) 
                {
                    didOverlap = AABBCollisionResponse((ObjectAABB*)PointerA, (ObjectAABB*)PointerB);
                }
                else if (shapeofA == CIRCLE && shapeofB == AABB) 
                {
                    didOverlap = CircleAABBCollisionResponse((ObjectCircle*)PointerA, (ObjectAABB*)PointerB);
                }
                else if (shapeofA == AABB && shapeofB == CIRCLE) 
                {
                    didOverlap = CircleAABBCollisionResponse((ObjectCircle*)PointerB, (ObjectAABB*)PointerA);
                }
                if (didOverlap) 
                {
                    if (PointerA->Shape() == CIRCLE && ((ObjectCircle*)PointerA)->isPig) 
                    {
                        if (PointerB->Shape() == CIRCLE && !((ObjectCircle*)PointerB)->isPig) 
                        {
                            ((ObjectCircle*)PointerA)->toRemove = true;
                        }
                        if (PointerB->Shape() == AABB && ((ObjectAABB*)PointerB)->isBullet) 
                        {
                            ((ObjectCircle*)PointerA)->toRemove = true;
                        }
                    }

                    if (PointerB->Shape() == CIRCLE && ((ObjectCircle*)PointerB)->isPig) 
                    {
                        if (PointerA->Shape() == CIRCLE && !((ObjectCircle*)PointerA)->isPig) 
                        {
                            ((ObjectCircle*)PointerB)->toRemove = true;
                        }
                        if (PointerA->Shape() == AABB && ((ObjectAABB*)PointerA)->isBullet) 
                        {
                            ((ObjectCircle*)PointerB)->toRemove = true;
                        }
                    }
                }




                if (shapeofA == HALF_SPACE) 
                {
                    PointerA->colour = RED;
                }
                else if (shapeofB == HALF_SPACE) 
                {
                    PointerB->colour = RED;
                }

            }
        }

    }
};
PhysicsSimulation ps;
ObjectHalfSpace halfSpace;
ObjectHalfSpace halfSpace2;
bool CircleHalfSpaceCollisionResponse(ObjectCircle* circle, ObjectHalfSpace* halfSpace) 
{
    Vector2 displacement = circle->position - halfSpace->position;
    float dot = Vector2DotProduct(displacement, halfSpace->getNormal());
    Vector2 vectorProjection = halfSpace->getNormal() * dot;
    float overlap = circle->radius - dot;
    if (overlap > 0) 
    {
        Vector2 mtv = halfSpace->getNormal() * overlap;
        circle->position += mtv;
        Vector2 fGravity = ps.gravityAcceleration * circle->mass;
        Vector2 fqPerp = halfSpace->getNormal() * Vector2DotProduct(fGravity, halfSpace->getNormal());
        Vector2 fNormal = fqPerp * -1;
        circle->netForce += fNormal;
        DrawLineEx(circle->position, circle->position + fNormal, 1, GREEN);       
        float u = circle->grippiness * halfSpace->grippiness;
        float frictionMagnitudeMax = u * Vector2Length(fNormal);

        Vector2 fgPara = fGravity - fqPerp;
        Vector2 frictionDirection = Vector2Normalize(fgPara) * -1;

        float fgParaMag = Vector2Length(fgPara);
        float clampedFriction = min(frictionMagnitudeMax, fgParaMag);
        Vector2 fFriction = frictionDirection * clampedFriction;

        float frictionForceLength = Vector2Length(fFriction);

        circle->netForce += fFriction;
        DrawLineEx(circle->position, circle->position + fFriction, 2, ORANGE);

        float closingVelocity1D = Vector2DotProduct(circle->velocity, halfSpace->getNormal());
        if (closingVelocity1D >= 0) return true;

        float restitution = circle->bounciness * halfSpace->bounciness;
		circle->velocity += halfSpace->getNormal() * (1.0f + restitution) * -closingVelocity1D;

        return true;
    }
    else 
    {
        return false;
    }
}

void cleanup() {
    for (int i = 0; i < ps.objekts.size(); i++) 
    {
        Object* o = ps.objekts[i];

        if (o->Shape() == CIRCLE) 
        {
            ObjectCircle* c = (ObjectCircle*)o;

            if (c->isPig) 
            {
                if (c->toRemove) 
                {
                    auto it = ps.objekts.begin() + i;
                    delete* it;
                    ps.objekts.erase(it);
                    i--;
                }
                continue;
            }
        }

        if (o->position.y > GetScreenHeight() ||
            o->position.y < 0 ||
            o->position.x > GetScreenWidth() ||
            o->position.x < 0) 
        {

            auto it = ps.objekts.begin() + i;
            delete* it;
            ps.objekts.erase(it);
            i--;
            continue;
        }
    }
}

void update() 
{
    dt = 1.0f / TARGET_FPS;
    stime += dt;

    ps.update();
    if (IsKeyPressed(KEY_Y)) 
    {
        currentBirdType = 1 - currentBirdType;
    }

    Vector2 mouse = GetMousePosition();

    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON))
    {
        dragging = true;
        dragStart = mouse;
        dragEnd = mouse;
    }

    if (dragging && IsMouseButtonDown(MOUSE_LEFT_BUTTON))
    {
        dragEnd = mouse;
    }

    if (dragging && IsMouseButtonReleased(MOUSE_LEFT_BUTTON))
    {
        dragging = false;
        Vector2 diff = dragStart - dragEnd;
        float power = Vector2Length(diff);
        if (power > maxPower) power = maxPower;
        Vector2 dir = Vector2Normalize(diff);
        Vector2 launchVel = dir * power;

        if (currentBirdType == 0) 
        {
            ObjectCircle* bird = new ObjectCircle;
            bird->position = dragStart;
            bird->velocity = launchVel;
            bird->radius = (float)((rand() % 25) + 5);
            bird->mass = 5;
            bird->bounciness = restitutionCoefficient;
            bird->colour = YELLOW;
            ps.add(bird);
        }
        else {
            ObjectAABB* box = new ObjectAABB(dragStart, { 40,40 }, false, BLUE, 20);
            box->velocity = launchVel;
            box->isBullet = true;
            ps.add(box);
        }
    }

    if (IsKeyPressed(KEY_SPACE)) 
    {
        ObjectCircle* bird = new ObjectCircle;
        bird->position = { 200, (float)GetScreenHeight() - 200 };
        bird->velocity = { (float)cos(angle * DEG2RAD) * speed, (float)-sin(angle * DEG2RAD) * speed };
        bird->radius = (float)((rand() % 25) + 5);
        bird->mass = sphereMass;
		bird->bounciness = restitutionCoefficient;
        ps.add(bird);

    }
    if (IsKeyPressed(KEY_A)) {
        ObjectAABB* box = new ObjectAABB({ 200, (float)GetScreenHeight() - 200 },{ 40, 40 },false,BLUE, 2.0f);
        box->velocity = { (float)cos(angle * DEG2RAD) * speed, (float)-sin(angle * DEG2RAD) * speed};
        ps.add(box);
    }

    cleanup();
}
void Draw() {

    BeginDrawing();
    ClearBackground(BLACK);
    DrawText("Conor DArcy 101446834", 10, float(GetScreenHeight() - 50), 30, GREEN);

    GuiSliderBar(Rectangle{ 10,15,1000,20 }, "", TextFormat("%.2f", stime), &stime, 0, 240);
    DrawText(TextFormat("Time: %6.2f", stime), GetScreenWidth() - 180, 10, 30, WHITE);

    GuiSliderBar(Rectangle{ 10,60,200,20 }, "Speed", TextFormat("Speed: %.0f", speed), &speed, 0, 1000);
    GuiSliderBar(Rectangle{ 10,90,200,20 }, "Angle", TextFormat("Angle: %.0f", angle), &angle, -90, 90);
    GuiSliderBar(Rectangle{ 10,120,200,20 }, "Gravity", TextFormat("Gravity: %.0f", ps.gravityAcceleration.y), &ps.gravityAcceleration.y, -180, 900);
    DrawText(TextFormat("Objects: %i", ps.objekts.size()), 10, 160, 30, WHITE);

    Vector2 startPos = { 200, GetScreenHeight() - 200 };
    Vector2 velocity1 = { cos(angle * DEG2RAD) * speed, -sin(angle * DEG2RAD) * speed };

    DrawLineEx(startPos, startPos + velocity1, 5, RED);

    //for half space
    GuiSliderBar(Rectangle{ 80,200,240,20 }, "X", TextFormat("%.0f", halfSpace.position.x), &halfSpace.position.x, 0, GetScreenWidth());
    GuiSliderBar(Rectangle{ 380,200,240,20 }, "Y", TextFormat("%.0f", halfSpace.position.y), &halfSpace.position.y, 0, GetScreenHeight());



    float halfSpaceAngle = halfSpace.getRotation();
    GuiSliderBar(Rectangle{ 780,200,100,20 }, "Rotation", TextFormat("%.0f", halfSpace.getRotation()), &halfSpaceAngle, -360, 360);
    halfSpace.setRotationDegree(halfSpaceAngle);
    GuiSliderBar(Rectangle{ 80,240,200,20 }, "u", TextFormat("%.1f", coefficientOfFriction), &coefficientOfFriction, 0, 1);
    GuiSliderBar(Rectangle{ 80,280,200,20 }, "mass", TextFormat("%.0f", sphereMass), &sphereMass, 0, 10);


    GuiSliderBar(Rectangle{ 80,320,200,20 }, "restitution", TextFormat("%.1f", restitutionCoefficient), &restitutionCoefficient, 0, 1);
    for (int i = 0;i < ps.objekts.size();i++) 
    {
        ps.objekts[i]->draw();

    }
    halfSpace.draw();
    if (dragging) 
    {
        DrawLineEx(dragStart, dragEnd, 4, RED);
        DrawCircleV(dragStart, 12, YELLOW);
        DrawCircleV(dragEnd, 8, WHITE);
    }
    DrawText(currentBirdType == 0 ? "Bird: CIRCLE (press Y to switch)": "Bird: AABB (press Y to switch)", 10, 450, 20, WHITE);

    EndDrawing();
}
int main() 
{
    InitWindow(InitialWidth, InitialHeight, "Sphere-Sphere Overlap");
    GuiLoadStyleDefault();
    halfSpace.isStatic = true;
    halfSpace.position = { 500,900 };
    halfSpace.setRotationDegree(0);
    ps.add(&halfSpace);

    ObjectAABB* ground = new ObjectAABB({ 800, 850 }, { 2000, 50 }, true, RED, 999999.0f);
    ps.add(ground);
    // Tower 
    ObjectAABB* box1 = new ObjectAABB({ 1000, 750 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box2 = new ObjectAABB({ 1000, 650 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box3 = new ObjectAABB({ 1000, 550 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box4 = new ObjectAABB({ 1500, 750 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box5 = new ObjectAABB({ 1500, 650 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box6 = new ObjectAABB({ 1500, 550 }, { 70, 100 }, false, DARKGRAY, 5.0f);
    ObjectAABB* box7 = new ObjectAABB({ 1250, 450 }, { 600, 70 }, false, DARKGRAY, 5.0f);
    ps.add(box1);
    ps.add(box2);
    ps.add(box3);
    ps.add(box4);
    ps.add(box5);
    ps.add(box6);
	ps.add(box7);
    ObjectCircle* circleA = new ObjectCircle();
    ObjectCircle* circleB = new ObjectCircle();
    ObjectCircle* circleC = new ObjectCircle();
    circleA->position = { 1250, 350 };
    circleA->radius = 50;
    circleA->mass = 1;
    circleA->colour = GREEN;
    circleB->position = { 1150, 750 };
    circleB->radius = 50;
    circleB->mass = 1;
    circleB->colour = GREEN;
    circleC->position = { 1350, 750 };
    circleC->radius = 50;
    circleC->mass = 1;
    circleC->colour = GREEN;
    circleA->isPig = true;

    circleB->isPig = true;

    circleC->isPig = true;


	ps.add(circleA);
	ps.add(circleB);
	ps.add(circleC);
    SetTargetFPS(TARGET_FPS);

    while (!WindowShouldClose()) {
        update();
        Draw();
    }
    CloseWindow();
    return 0;
}
