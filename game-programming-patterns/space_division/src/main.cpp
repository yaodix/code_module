#include "ball.h"
#include "util.h"
#include "quadtree.h"
#include <iostream>
#include <random>
#include <vector>
#include <ctime>
#include <SFML/Graphics.hpp>



void GeneratePopulation(int n, std::vector<Ball *> &ball_vec) {
    for (int i = 0; i < n;) {
        int x = rand() % (kScreenWidth - 2 * kBallRadius - 5);
        int y = rand() % (kScreenHeight - 2 * kBallRadius - 5);
        bool good_position = true;
        for (const auto &b: ball_vec) {
            double d = DistSquared(static_cast<double>(x), static_cast<double>(y), b->GetX(), b->GetY());
            if (d <= kBallRadius * kBallRadius) {
                good_position = false;
                break;
            }
        }
        if (!good_position) {
            continue;
        }
        double dx = 0;
        double dy = 0;
        int x_rand = (rand() % (2 * kModulo)) - kModulo;
        int y_rand = (rand() % (2 * kModulo)) - kModulo;
        dx = x_rand > 0 ? kBallDx : -kBallDx;
        dy = y_rand > 0 ? kBallDy : -kBallDy;
        sf::Color color = sf::Color::Black;
        if (i < kNumRedBalls) {
            color = sf::Color::Red;
        }
        Ball *new_ball = new Ball(x, y, dx, dy, kBallRadius, color);
        ball_vec.push_back(new_ball);
        ++i;
    }
}





void DrawQtree(Quadtree *root, sf::RenderWindow &window) {
    Quadtree *curr = root;
    Quadtree *NW = curr->GetNW();
    if (NW) {
        DrawQtree(NW, window);
        DrawQtree(curr->GetNE(), window);
        DrawQtree(curr->GetSW(), window);
        DrawQtree(curr->GetSE(), window);
    } else {
        window.draw(curr->GetRectShape());
    }
}


void QtreePrintSize(Quadtree *root) {
    Quadtree *NW = root->GetNW();
    if (NW != nullptr) {
        QtreePrintSize(NW);
        QtreePrintSize(root->GetNE());
        QtreePrintSize(root->GetSW());
        QtreePrintSize(root->GetSE());
    } else {
        auto r = root->GetRect();
        std::cout << std::endl;
        std::cout << "rect: (" << r.left << ", " << r.top <<
        ", " << r.width << ", " << r.height <<
        "), SIZE: " << root->GetSize() << std::endl;
        std::cout << "children: " << std::endl;
        for (const auto &node: root->GetChildren()) {
            std::cout << "(" << node->GetX() << ", " <<
            node->GetY() << ")" << std::endl;
        }
    }
}


void CheckCollisionsBruteforce(std::vector<Ball *> &ball_vec) {
    for (size_t i = 0; i < ball_vec.size(); ++i) {
        for (size_t j = 0; j < ball_vec.size(); ++j) {
            if (i == j) {
                continue;
            }
            ball_vec[i]->CheckCollisionWithBall(ball_vec[j]);
            ball_vec[i]->CheckCollisionWithMap(kHorizontal);
            ball_vec[i]->CheckCollisionWithMap(kVertical);
        }
    }
}


double GetAverageSpeed(std::vector<Ball *> &ball_vec) {
    double res = 0;
    for (const auto &b: ball_vec) {
        res += fabs(b->GetDx()) + fabs(b->GetDy());
    }
    return res / (2 * ball_vec.size());
}

double GetMaxSpeed(std::vector<Ball *> &ball_vec) {
    double max_speed = 0;
    for (const auto &b: ball_vec) {
        double dx = b->GetDx();
        double dy = b->GetDy();
        double curr_speed = dx * dx + dy * dy;
        if (curr_speed > max_speed) {
            max_speed = curr_speed;
        }
    }
    return max_speed;
}



int main() {
    srand(time(nullptr));
    std::vector<Ball *> ball_vec;
    GeneratePopulation(kNumBalls, ball_vec);

    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;
    sf::RenderWindow window(sf::VideoMode(kScreenWidth, kScreenHeight), "Quadtree collision detection", sf::Style::Default, settings);
    window.setFramerateLimit(kFrameRateLimit);
    sf::Clock clock;

    int num_collisions = 0;

    long long output_dt = 0;

    while (window.isOpen()) {
        Quadtree *qtree = new Quadtree(0, 0, kScreenWidth, kScreenHeight);
        for (auto b: ball_vec) {
            qtree->InsertElement(b);
        }
        output_dt += clock.getElapsedTime().asMilliseconds();
        auto time_elapsed = clock.getElapsedTime().asMicroseconds();
        double dt = static_cast<double>(time_elapsed);
        int fps_count = static_cast<int>(1 / (dt / 1000000));

        dt /= kTimeAdjustment;
        clock.restart();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.clear(sf::Color::White);
        for (auto &b: ball_vec) {
            b->Update(dt);
        }
        //double max_speed = GetMaxSpeed(ball_vec);
        //max_speed = sqrt(max_speed);
        //double avg_speed = GetAverageSpeed(ball_vec);
        //std::cout << "AVG: " << avg_abs_speed << std::endl;
        //for(auto &b: ball_vec){
            //b->update_temperature_color(2 * average_speed, average_speed);
        //}
        //CheckCollisionsBruteforce(ball_vec);
        qtree->QtreeCheckCollisions(num_collisions);
        for (size_t i = 0; i < ball_vec.size(); ++i) {
            ball_vec[i]->CheckCollisionWithMap(kHorizontal);
            ball_vec[i]->CheckCollisionWithMap(kVertical);
        }

        if (kPrintStatsToConsole) {
            if (output_dt > kOutputTimeCutoff) {
                std::cout << "num_checks bruteforce: " << kNumBalls * (kNumBalls - 1) << ", ";
                std::cout << "num_checks quadtree: " << num_collisions;
                std::cout << ", FPS: " << fps_count << std::endl;
                num_collisions = 0;
                output_dt = 0;
            }
        } 
        if (kDrawQuadtreeBoundaries) {
            DrawQtree(qtree, window);
        }

        for (auto &b: ball_vec) {
            b->GetShape().setPosition(sf::Vector2f(b->GetX(), b->GetY()));
            window.draw(b->GetShape());
        }
        window.display();
        qtree->QtreeFreeMemory();
    }
    return 0;
}
