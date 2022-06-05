#ifndef QUADTREE_H_
#define QUADTREE_H_

#include "ball.h"
#include <vector>

class Quadtree {
public:
    Quadtree(int left, int top, int width, int height);
    bool InsertElement(Ball *b);
    void Subdivide(Quadtree *root);
    int GetSize() const {
        return children_.size();
    }
    Quadtree *GetNW() const {
        return NW_;
    }
    Quadtree *GetNE() const {
        return NE_;
    }
    Quadtree *GetSW() const {
        return SW_;
    }
    Quadtree *GetSE() const {
        return SE_;
    }
    sf::IntRect GetRect() const {
        return rect_;
    }
    sf::RectangleShape GetRectShape() const {
        return rect_shape_;
    }
    std::vector<Ball *> GetChildren() const {
        return children_;
    }
    void QtreeCheckCollisions(int &num_collisions);
    void QtreeFreeMemory();
private:
    std::vector<Ball *> children_;
    Quadtree *NW_ = nullptr;
    Quadtree *NE_ = nullptr;
    Quadtree *SW_ = nullptr;
    Quadtree *SE_ = nullptr;
    sf::RectangleShape rect_shape_;
    sf::IntRect rect_;
};

#endif // QUADTREE_H_
