#include "register_types.hpp"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "nodes/boid_animated_sprite2d.hpp"
#include "nodes/boid_manager_node.hpp"
#include "nodes/boid_physics_body2d.hpp"
#include "nodes/boid_rigid_body2d.hpp"
#include "nodes/boid_sprite2d.hpp"
#include "player.hpp"
#include "storage_system/inventory_container.hpp"

using godot::MODULE_INITIALIZATION_LEVEL_SCENE;

void initialize_example_module(godot::ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    GDREGISTER_CLASS(godot::Player);
    GDREGISTER_CLASS(godot::InventoryContainer);
    GDREGISTER_CLASS(godot::ItemBlueprint);
    GDREGISTER_CLASS(godot::BoidManagerNode);
    GDREGISTER_CLASS(godot::BoidClusterNode);
    GDREGISTER_CLASS(godot::BoidAnimatedSprite2D);
    GDREGISTER_CLASS(godot::BoidSprite2D);
    GDREGISTER_CLASS(godot::BoidPhysicsBody2D);
    GDREGISTER_CLASS(godot::BoidRigidBody2D);
}

void uninitialize_example_module(godot::ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {
// Initialization.
GDExtensionBool GDE_EXPORT example_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
                                                const GDExtensionClassLibraryPtr p_library,
                                                GDExtensionInitialization* r_initialization) {
    godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

    init_obj.register_initializer(initialize_example_module);
    init_obj.register_terminator(uninitialize_example_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

    return init_obj.init();
}
}