#ifndef MYMAP_H
#define MYMAP_H
#include <deque>

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

    bool is_red(Node* p) const {
        if (!p) return false; // Null links are black
        return p->color == Color::Red;
    }

    Node* rotate_left(Node* h) {
        Node* x = h->right;
        h->right = x->left;
        x->left = h;
        x->color = h->color;
        h->color = Color::Red;
        return x;
    }

    Node* rotate_right(Node* h) {
        Node* x = h->left;
        h->left = x->right;
        x->right = h;
        x->color = h->color;
        h->color = Color::Red;
        return x;
    }

    void move_red_up(Node* h) {
        h->color = Color::Red;
        h->left->color = Color::Black;
        h->right->color = Color::Black;
    }

    Node* find_node(Node* p, K const& k) const {
        if (!p) return p;
        if (p->key < k) return find_node(p->right, k);
        if (p->key > k) return find_node(p->left, k);
        return p;
    }

    Node* insert_node(Node* node, K const& k, V const& v) {
        if (!node) {
            nodes.emplace_back(k, v);
            return &nodes.back();
        }
        if (node->key < k) {
            auto new_right = insert_node(node->right, k, v);
            node->right = new_right;
        } else if (node->key > k) {
            auto new_left = insert_node(node->left, k, v);
            node->left = new_left;
        } else {
            node->key = k;
            node->value = v;
        }

        if (is_red(node->right) && !is_red(node->left)) node = rotate_left(node);
        if (is_red(node->left) && is_red(node->left->left)) node = rotate_right(node);
        if (is_red(node->left) && is_red(node->right)) move_red_up(node);

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
        root = insert_node(root, key, value);
        root->color = Color::Black;
    }
    V const* find(K const& key) const {
        if (Node* p = find_node(root, key)) return &p->value;
        return nullptr;
    }
    V* find(K const& key) {
        if (Node* p = find_node(root, key)) return &p->value;
        return nullptr;
    }
    MyMap(MyMap const&) = delete;
    MyMap& operator=(MyMap const&) = delete;
};

#endif
