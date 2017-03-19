#ifndef MYMAP_H
#define MYMAP_H
#include <cstdint>
#include <deque>
#include <random>

// This BST implementation is faster than std::map. Benchmark yourself if you
// don't believe me.

template<typename K, typename V>
class MyMap {
private:
    struct Node {
        K key;
        V value;
        Node *left, *right;
        uint64_t priority;
        Node(K const& key, V const& value) : key(key), value(value), left{nullptr}, right{nullptr}, priority() {
            static std::mt19937_64 generator(0);
            priority = generator();
        }
    };

    typedef std::deque<Node> Arena;
    // The only reason we use std::deque is because we want stable references
    // yet clustering behavior for cache locality. For stability of references,
    // see C++11 [deque.modifiers] 23.3.3.4/1: An insertion at either end of the
    // deque invalidates all the iterators to the deque, but has no effect on
    // the validity of references to elements of the deque.
    Arena nodes;
    Node* root;

    static Node* rotateLeft(Node* h) {
        Node* x = h->right;
        h->right = x->left;
        x->left = h;
        return x;
    }

    static Node* rotateRight(Node* h) {
        Node* x = h->left;
        h->left = x->right;
        x->right = h;
        return x;
    }

    static Node* findNode(Node* p, K const& k) {
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
            if (newRight->priority < node->priority) node = rotateLeft(node);
        } else if (k < node->key) {
            auto newLeft = insertNode(node->left, k, v);
            node->left = newLeft;
            if (newLeft->priority < node->priority) node = rotateRight(node);
        } else {
            node->key = k;
            node->value = v;
        }

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
    void associate(const K& key, const V& value) { root = insertNode(root, key, value); }
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
