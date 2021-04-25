# Pioneer 3-DX

Simulated in CoppeliaSim, Model provided in the standard EDU installation.

Implemented Switching based Sliding Mode Controller. Reference Paper given in the 'paper' directory.

Current Issues:

-> Linear Control develops an offset over time and loses control if the desired angular velocity is zero.

Directories:

control: Sliding Mode Controller Implementation

controller: Diff Drive Implementation

P3-DX: BringUp folder

simulation: Scenes + Lua Scripts for CoppeliaSim

