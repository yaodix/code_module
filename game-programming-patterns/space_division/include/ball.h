#ifndef BALL_H_
#define BALL_H_

#include "config.h"
#include <random>
#include <vector>
#include <SFML/Graphics.hpp>

enum direction {
    kHorizontal,
    kVertical
};

class Ball {
public:
    Ball(double x, double y, double dx, double dy, double r = kBallRadius, sf::Color color = sf::Color::Black):
        x_(x), y_(y), dx_(dx), dy_(dy), r_(r), shape_(r) {
        shape_.setOrigin(sf::Vector2f(GetR(), GetR()));
        shape_.setPosition(x, y);
        shape_.setFillColor(color);
        shape_.setOutlineThickness(kBallOutlineThickness);
        shape_.setOutlineColor(sf::Color::Black);

    }
    double GetX() const {
        return x_;
    }
    double GetY() const {
        return y_;
    }
    double GetR() const {
        return r_;
    }
    void SetX(double x) {
        x_ = x;
    }
    void SetY(double y) {
        y_ = y;
    }
    double GetDx() const {
        return dx_;
    }
    double GetDy() const {
        return dy_;
    }
    void SetDx(double dx) {
        dx_ = dx;
    }
    void SetDy(double dy) {
        dy_ = dy;
    }
    const sf::CircleShape& GetShape() const {
        return shape_;
    }
    sf::CircleShape& GetShape() {
        return shape_;
    }
    void Update(double dt);
    void UpdateTemperatureColor(double max_speed, double avg_abs_speed);
    void CheckCollisionWithBall(Ball *ball);
    void CheckCollisionWithMap(direction dir);


private:
    double x_ = 0.0;  // 圆心坐标x
    double y_ = 0.0;
    double dx_ = 0.0;  // x方向移动速度
    double dy_ = 0.0;
    double r_ = 0.0;   // 半径
    sf::CircleShape shape_;
    sf::Text speed_text_;
    static sf::Font ball_font_;
};
#endif // BALL_H_
