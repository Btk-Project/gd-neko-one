#pragma once

#include "inventory_container.hpp"

#include <godot_cpp/classes/item_list.hpp>

namespace godot {
class InventoryView : public ItemList {
    // NOLINTBEGIN
    GDCLASS(InventoryView, ItemList)
    // NOLINTEND

public:
    InventoryView(const InventoryView&)               = default;
    InventoryView(InventoryView&&)                    = delete;
    auto operator=(InventoryView&&) -> InventoryView& = delete;
    ~InventoryView();

    void set_data(const Ref<InventoryContainer>& container);
    void _ready() override;

    static void _bind_methods();
private:
    void _on_item_list_item_selected(int64_t index);
    void _on_item_list_item_activated(int64_t index);
    void _on_item_list_item_long_pressed(int64_t index);

    Ref<InventoryContainer> container;
};
} // namespace godot