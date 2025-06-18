#include "item_blueprint.hpp"

namespace godot {
void ItemBlueprint::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_display_name", "name"), &ItemBlueprint::set_display_name);
    ClassDB::bind_method(D_METHOD("get_display_name"), &ItemBlueprint::get_display_name);
    ClassDB::bind_method(D_METHOD("set_description", "description"), &ItemBlueprint::set_description);
    ClassDB::bind_method(D_METHOD("get_description"), &ItemBlueprint::get_description);
    ClassDB::bind_method(D_METHOD("set_item_id", "item_id"), &ItemBlueprint::set_item_id);
    ClassDB::bind_method(D_METHOD("get_item_id"), &ItemBlueprint::get_item_id);
    ClassDB::bind_method(D_METHOD("set_max_stack_size", "max_stack_size"), &ItemBlueprint::set_max_stack_size);
    ClassDB::bind_method(D_METHOD("get_max_stack_size"), &ItemBlueprint::get_max_stack_size);
    ClassDB::bind_method(D_METHOD("set_item_type", "item_type"), &ItemBlueprint::set_item_type);
    ClassDB::bind_method(D_METHOD("get_item_type"), &ItemBlueprint::get_item_type);
    ClassDB::bind_method(D_METHOD("set_custom_properties", "custom_properties"), &ItemBlueprint::set_custom_properties);
    ClassDB::bind_method(D_METHOD("get_custom_properties"), &ItemBlueprint::get_custom_properties);
    ClassDB::bind_method(D_METHOD("set_quality", "quality"), &ItemBlueprint::set_quality);
    ClassDB::bind_method(D_METHOD("get_quality"), &ItemBlueprint::get_quality);
}

// Setters and Getters

void ItemBlueprint::set_display_name(const String& p_display_name) { display_name = p_display_name; }
auto ItemBlueprint::get_display_name() const -> String { return display_name; }
void ItemBlueprint::set_description(const String& p_description) { description = p_description; }
auto ItemBlueprint::get_description() const -> String { return description; }
void ItemBlueprint::set_item_id(const String& p_id) { item_id = p_id; }
auto ItemBlueprint::get_item_id() const -> String { return item_id; }
auto ItemBlueprint::set_max_stack_size(int p_max_stack_size) -> void { max_stack_size = p_max_stack_size; }
auto ItemBlueprint::get_max_stack_size() const -> int { return max_stack_size; }
auto ItemBlueprint::set_item_type(const StringName& p_item_type) -> void { item_type = p_item_type; }
auto ItemBlueprint::get_item_type() const -> StringName { return item_type; }
auto ItemBlueprint::set_custom_properties(const Dictionary& p_custom_properties) -> void {
    custom_properties = p_custom_properties;
}
auto ItemBlueprint::get_custom_properties() const -> Dictionary { return custom_properties; }
auto ItemBlueprint::set_quality(int p_quality) -> void { quality = p_quality; }
auto ItemBlueprint::get_quality() const -> int { return quality; }

} // namespace godot