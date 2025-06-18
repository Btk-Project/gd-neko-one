#pragma once

#include <godot_cpp/classes/resource.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/string_name.hpp>

namespace godot {

class ItemBlueprint : public Resource {
    // NOLINTBEGIN
    GDCLASS(ItemBlueprint, Resource);
    // NOLINTEND

private:
    String item_id;
    String display_name;
    String description;
    // Ref<Texture2D> icon;
    int max_stack_size = 99;
    int quality        = 1;
    StringName item_type; // e.g., "weapon", "armor", "consumable"
    Dictionary custom_properties;

protected:
    static void _bind_methods();

public:
    ItemBlueprint()                                   = default;
    ItemBlueprint(const ItemBlueprint&)               = default;
    ItemBlueprint(ItemBlueprint&&)                    = delete;
    auto operator=(ItemBlueprint&&) -> ItemBlueprint& = delete;
    ~ItemBlueprint()                                  = default;

    // Setters and Getters
    void set_display_name(const String& p_display_name);
    auto get_display_name() const -> String;
    void set_description(const String& p_description);
    auto get_description() const -> String;
    void set_item_id(const String& p_id);
    auto get_item_id() const -> String;
    auto set_max_stack_size(int p_max_stack_size) -> void;
    auto get_max_stack_size() const -> int;
    auto set_item_type(const StringName& p_item_type) -> void;
    auto get_item_type() const -> StringName;
    auto set_custom_properties(const Dictionary& p_custom_properties) -> void;
    auto get_custom_properties() const -> Dictionary;
    auto set_quality(int p_quality) -> void;
    auto get_quality() const -> int;
};
} // namespace godot