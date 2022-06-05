#include "ball.h"
#include "util.h"
#include <iostream>


void Ball::Update(double dt) {
    SetX(GetX() + dt * GetDx());
    SetY(GetY() + dt * GetDy());
}


void Ball::CheckCollisionWithMap(direction dir) {
    if (dir == kHorizontal) {
        if (GetX() - GetR() <= 0) {
            SetX(GetR() + kCollisionOffset);
            SetDx(-GetDx());
        } else if (GetX() + GetR() >= kScreenWidth) {
            SetX(kScreenWidth - GetR() - kCollisionOffset);
            SetDx(-GetDx());
        }
    } else if (dir == kVertical) {
        if (GetY() - GetR() <= 0) {
            SetY(GetR() + kCollisionOffset);
            SetDy(-GetDy());
        } else if (GetY() + GetR() >= kScreenHeight) {
            SetY(kScreenHeight - GetR() - kCollisionOffset);
            SetDy(-GetDy());
        }
    }
}


void Ball::CheckCollisionWithBall(Ball *b) {
    double d = DistSquared(GetX(), GetY(), b->GetX(), b->GetY());
    double sum_radius = GetR() + b->GetR();
    double cutoff = sum_radius * sum_radius;
    if (d > cutoff) {
        return;
    }
    double shift = GetR() + b->GetR() - Dist(GetX(), GetY(), b->GetX(), b->GetY());
    double v_x = GetX() - b->GetX();
    double v_y = GetY() - b->GetY();
    double len = Length(v_x, v_y);
    v_x /= len;
    v_y /= len;
    v_x *= shift;
    v_y *= shift;
    SetX(GetX() + v_x);
    SetY(GetY() + v_y);

    double center_x = GetX() - b->GetX();
    double center_y = GetY() - b->GetY();
    double dot_prod = DotProduct(GetDx() - b->GetDx(), GetDy() - b->GetDy(), center_x, center_y);
    double d_centers = DistSquared(GetX(), GetY(), b->GetX(), b->GetY());

    SetDx(GetDx() - center_x * dot_prod / d_centers);
    b->SetDx(b->GetDx() + center_x * dot_prod / d_centers);
    SetDy(GetDy() - center_y * dot_prod / d_centers);
    b->SetDy(b->GetDy() + center_y * dot_prod / d_centers);
}


void Ball::UpdateTemperatureColor(double max_speed, double avg_speed) {
    double dx = GetDx();
    double dy = GetDy();
    double speed = sqrt(dx * dx + dy * dy);
    if (speed <= avg_speed) {//the ball is moving slower than average
        double c = speed / avg_speed;
        GetShape().setFillColor(sf::Color(c * 255, c * 255, 255));
    } else {
        double c = speed / max_speed;
        c = 2 - c;
        GetShape().setFillColor(sf::Color(255, c * 255, c * 255));
    }
}
