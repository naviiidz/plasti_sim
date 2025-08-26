# Can Models - Texture and Sinking Issues Fixed

## Problems Identified

### 1. **Textures Not Showing**
**Root Cause**: Incorrect material script URI paths
- **Original**: `<uri>model://can_coke/materials/scripts</uri>`
- **Issue**: Missing file:// protocol and incorrect path structure

**Fix Applied**:
```xml
<script>
  <uri>file://can_coke/materials/scripts/coke.material</uri>
  <name>coke</name>
</script>
```

### 2. **Cans Sinking**
**Root Causes**:
- **Heavy Mass**: 350g + 15g = 365g total (too heavy to float)
- **No Buoyancy Plugin**: Missing floating physics
- **Wrong Plugin**: Some had custom plugins that weren't working

**Fix Applied**:
- **Reduced Mass**: 120g + 8g = 128g total (realistic for aluminum can)
- **Added VRX Buoyancy**: Standard `vrx::PolyhedraBuoyancyDrag` plugin
- **Proper Buoyancy Geometry**: Cylinder approximation for stable floating

## Models Fixed

### ✅ **can_coke** - COMPLETE
- Fixed texture loading (`file://can_coke/materials/scripts/coke.material`)
- Added VRX buoyancy plugin
- Reduced mass from 365g to 128g
- Added velocity decay for stability
- Proper collision surface properties

### ✅ **can_pepsi** - COMPLETE  
- Fixed texture loading (`file://can_pepsi/materials/scripts/pepsi.material`)
- Replaced custom plugins with VRX standard
- Reduced mass from 365g to 128g
- Added velocity decay for stability
- Proper collision surface properties

### ⏳ **can_fanta** - PENDING
- Same issues as above models
- Needs identical fixes

### ⏳ **can_sprite** - PENDING
- Same issues as above models
- Needs identical fixes

### ⏳ **tuna_can** - PENDING
- May have different geometry but same physics issues

## Technical Details

### **New Physics Configuration**
```xml
<!-- VRX Buoyancy Plugin -->
<plugin name="vrx::PolyhedraBuoyancyDrag" filename="libPolyhedraBuoyancyDrag.so">
  <fluid_density>1000.0</fluid_density>  <!-- Freshwater -->
  <fluid_level>0.0</fluid_level>
  <linear_drag>8.0</linear_drag>         <!-- Moderate drag -->
  <angular_drag>1.5</angular_drag>
  
  <buoyancy name="can_buoyancy">
    <link_name>link_0</link_name>
    <pose>0 0 0 0 -1.67 0</pose>
    <geometry>
      <cylinder>
        <radius>0.032</radius>  <!-- 64mm diameter -->
        <length>0.12</length>   <!-- 120mm height -->
      </cylinder>
    </geometry>
  </buoyancy>
  
  <wavefield>
    <topic>/vrx/wavefield/parameters</topic>
  </wavefield>
</plugin>
```

### **Mass Distribution**
- **Main Body (link_0)**: 120g (body + liquid content)
- **Metal Parts (link_1)**: 8g (aluminum structure)
- **Total**: 128g (realistic for aluminum can with some liquid)

### **Material Loading Fix**
- **Protocol**: Added `file://` prefix
- **Path**: Direct path to .material file
- **Fallback**: Added ambient/diffuse properties for backup

## Expected Results

### ✅ **Floating Behavior**
- Cans should now float upright on water surface
- Stable floating with proper buoyancy
- Respond to waves via VRX wavefield integration

### ✅ **Texture Rendering**
- Brand textures (Coke, Pepsi) should display properly
- Material scripts load with correct file paths
- Fallback materials ensure visibility even if textures fail

### ✅ **Physics Stability**
- Velocity decay prevents endless drifting
- Proper collision surfaces for interaction
- Compatible with VRX marine environment

## Next Steps

To complete the fixes:
1. Apply same pattern to `can_fanta` and `can_sprite`
2. Check `tuna_can` geometry (different shape, may need box buoyancy)
3. Test all models in VRX water environment
4. Verify texture loading for all brands

## Build Status
✅ **plastiSim package built successfully** with can_coke and can_pepsi fixes
