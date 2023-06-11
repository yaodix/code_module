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
            // std::cout << "kHorizontal befor dx dy " << GetDx() << " " << GetDy() << std::endl;
            SetDx(-GetDx());
            // std::cout << "colli dx dy " << GetDx() << " " << GetDy() << std::endl;
        } else if (GetX() + GetR() >= kScreenWidth) {
            SetX(kScreenWidth - GetR() - kCollisionOffset);
            // std::cout << "kHorizontal befor dx " << GetDx() << " " << GetDy() << std::endl;
            SetDx(-GetDx());
            // std::cout << "colli dx dy " << GetDx() << " " << GetDy() << std::endl;
        }
    } else if (dir == kVertical) {
        if (GetY() - GetR() <= 0) {
            SetY(GetR() + kCollisionOffset);
            // std::cout << "kVertical befor dx dy "  << GetDx() << " " << GetDy() << std::endl;
            SetDy(-GetDy());
            // std::cout << "colli dx dy "  << GetDx() << " " << GetDy() << std::endl;

        } else if (GetY() + GetR() >= kScreenHeight) {
            SetY(kScreenHeight - GetR() - kCollisionOffset);
            // std::cout << "kVertical befor dx dy "  << GetDx() << " " << GetDy() << std::endl;
            SetDy(-GetDy());
            // std::cout << "colli dx dy "  << GetDx() << " " << GetDy() << std::endl;

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
    // std::cout << "ball x y " << GetX() << " " << GetY() << " b x y " << b->GetX() << " " << b->GetY() << std::endl;
    // std::cout << "ball colli shift v_x v_y len " << shift << " " << v_x << " " << v_y << " " << len << std::endl;
    v_x /= len;
    v_y /= len;
    // std::cout << "/len vx vy " << v_x << " " << v_y << std::endl;
    v_x *= shift;
    v_y *= shift;
    // std::cout << "*shift vx vy " << v_x << " " << v_y << std::endl;
    SetX(GetX() + v_x);  // 当前ball从轻微碰撞恢复到未碰撞状态位置
    SetY(GetY() + v_y);  
    // std::cout << "ball x y " << GetX() << " " << GetY() << std::endl;

    double center_x = GetX() - b->GetX();  // 两个球心的向量
    double center_y = GetY() - b->GetY();
    double dot_prod = DotProduct(GetDx() - b->GetDx(), GetDy() - b->GetDy(), center_x, center_y);
    double d_centers = DistSquared(GetX(), GetY(), b->GetX(), b->GetY());
    // std::cout << "var2 center_x " << center_x << " " << center_y << " " << dot_prod << " " << d_centers << std::endl;

    SetDx(GetDx() - center_x * dot_prod / d_centers);
    SetDy(GetDy() - center_y * dot_prod / d_centers);
    // std::cout << "setDx " << GetDx() - center_x * dot_prod / d_centers <<std::endl;
    // std::cout << "setDy " << GetDy() - center_y * dot_prod / d_centers <<std::endl;

    b->SetDx(b->GetDx() + center_x * dot_prod / d_centers);
    b->SetDy(b->GetDy() + center_y * dot_prod / d_centers);
    // std::cout << "b setDx " << b->GetDx() + center_x * dot_prod / d_centers <<std::endl;
    // std::cout << "b setDy " << b->GetDy() + center_y * dot_prod / d_centers <<std::endl;
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
