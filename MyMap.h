#ifndef MYMAP_H
#define MYMAP_H
#include <deque>

// This BST implementation is faster than std::map. Benchmark yourself if you
// don't believe me.

template<typename K, typename V>
class MyMap {
private:
    enum class Color { Red, Black };
    struct Node {
        K key;
        V value;
        Node *left, *right;
        Color color;
        Node(K const& key, V const& value) : key(key), value(value), left{nullptr}, right{nullptr}, color(Color::Red) {}
    };

    typedef std::deque<Node> Arena;
    // The only reason we use std::deque is because we want stable references
    // yet clustering behavior for cache locality. For stability of references,
    // see C++11 [deque.modifiers] 23.3.3.4/1: An insertion at either end of the
    // deque invalidates all the iterators to the deque, but has no effect on
    // the validity of references to elements of the deque.
    Arena nodes;
    Node* root;

    bool isRed(Node* p) const {
        if (!p) return false; // Null links are black
        return p->color == Color::Red;
    }

    Node* rotateLeft(Node* h) {
        Node* x = h->right;
        h->right = x->left;
        x->left = h;
        x->color = h->color;
        h->color = Color::Red;
        return x;
    }

    Node* rotateRight(Node* h) {
        Node* x = h->left;
        h->left = x->right;
        x->right = h;
        x->color = h->color;
        h->color = Color::Red;
        return x;
    }

    void moveRedUp(Node* h) {
        h->color = Color::Red;
        h->left->color = Color::Black;
        h->right->color = Color::Black;
    }

    Node* findNode(Node* p, K const& k) const {
        if (!p) return p;
        if (p->key < k) return findNode(p->right, k);
        if (k < p->key) return findNode(p->left, k);
        return p;
    }

    Node* insertNode(Node* node, K const& k, V const& v) {
        if (!node) {
            nodes.emplace_back(k, v);
            return &nodes.back();
        }
        if (node->key < k) {
            auto newRight = insertNode(node->right, k, v);
            node->right = newRight;
        } else if (k < node->key) {
            auto newLeft = insertNode(node->left, k, v);
            node->left = newLeft;
        } else {
            node->key = k;
            node->value = v;
        }

        if (isRed(node->right) && !isRed(node->left)) node = rotateLeft(node);
        if (isRed(node->left) && isRed(node->left->left)) node = rotateRight(node);
        if (isRed(node->left) && isRed(node->right)) moveRedUp(node);

        return node;
    }

public:
    MyMap() : nodes{}, root{nullptr} {}
    ~MyMap() = default;
    void clear() {
        nodes.clear();
        root = nullptr;
    }
    int size() const { return nodes.size(); }
    void associate(const K& key, const V& value) {
        root = insertNode(root, key, value);
        root->color = Color::Black;
    }
    V const* find(K const& key) const {
        if (Node* p = findNode(root, key)) return &p->value;
        return nullptr;
    }
    V* find(K const& key) {
        if (Node* p = findNode(root, key)) return &p->value;
        return nullptr;
    }
    MyMap(MyMap const&) = delete;
    MyMap& operator=(MyMap const&) = delete;
    MyMap(MyMap&&) = default;
    MyMap& operator=(MyMap&&) = default;
};

#endif
