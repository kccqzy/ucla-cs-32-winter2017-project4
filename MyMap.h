// MyMap.h

#ifndef MYMAP_H
#define MYMAP_H
#include <deque>

template<typename K, typename V>
class MyMap {
public:
    MyMap() : nodes{}, root{-1} {}
    ~MyMap() = default;
    void clear() {
        nodes.clear();
        root = {-1};
    }
    int size() const { return nodes.size(); }

    MyMap(MyMap const&) = delete;
    MyMap& operator=(MyMap const&) = delete;

private:
    struct Ptr {
        int i;
        explicit operator bool() const { return i != -1; }
    };
    enum class Color { Red, Black };
    struct Node {
        K key;
        V value;
        Ptr left, right;
        Color color;
        Node(K const& key, V const& value, Color color) : key(key), value(value), left{-1}, right{-1}, color(color) {}
    };
    Node const& deref(Ptr i) const { return nodes[i.i]; }
    Node& deref(Ptr i) { return nodes[i.i]; }

    std::deque<Node> nodes;
    Ptr root;

    bool is_red(Ptr p) const {
        if (!p) return false; // Null links are black
        return deref(p).color == Color::Red;
    }

    Ptr rotate_left(Ptr h) {
        Ptr x = deref(h).right;
        deref(h).right = deref(x).left;
        deref(x).left = h;
        deref(x).color = deref(h).color;
        deref(h).color = Color::Red;
        return x;
    }

    Ptr rotate_right(Ptr h) {
        Ptr x = deref(h).left;
        deref(h).left = deref(x).right;
        deref(x).right = h;
        deref(x).color = deref(h).color;
        deref(h).color = Color::Red;
        return x;
    }

    void move_red_up(Ptr h) {
        deref(h).color = Color::Red;
        deref(deref(h).left).color = Color::Black;
        deref(deref(h).right).color = Color::Black;
    }

    Ptr find_node(Ptr p, K const& k) const {
        if (!p) return {-1};
        if (deref(p).key < k)
            return find_node(deref(p).right, k);
        else if (deref(p).key > k)
            return find_node(deref(p).left, k);
        else
            return p;
    }

    Ptr insert_node(Ptr node, K const& k, V const& v) {
        if (!node) {
            nodes.emplace_back(k, v, Color::Red);
            return {static_cast<int>(nodes.size() - 1)};
        }
        if (deref(node).key < k) {
            auto new_right = insert_node(deref(node).right, k, v);
            deref(node).right = new_right;
        } else if (deref(node).key > k) {
            auto new_left = insert_node(deref(node).left, k, v);
            deref(node).left = new_left;
        } else {
            deref(node).key = k;
            deref(node).value = v;
        }

        if (is_red(deref(node).right) && !is_red(deref(node).left)) node = rotate_left(node);
        if (is_red(deref(node).left) && is_red(deref(deref(node).left).left)) node = rotate_right(node);
        if (is_red(deref(node).left) && is_red(deref(node).right)) move_red_up(node);

        return node;
    }

public:
    void associate(const K& key, const V& value) {
        root = insert_node(root, key, value);
        deref(root).color = Color::Black;
    }
    V const* find(K const& key) const {
        if (Ptr p = find_node(root, key)) return &deref(p).value;
        return nullptr;
    }
    V* find(K const& key) {
        if (Ptr p = find_node(root, key)) return &deref(p).value;
        return nullptr;
    }
};

#endif
