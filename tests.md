Run these tests pertaining to code you may have altered in a commit


1. Test 1: Check functionality of three scene environment
 - Add cube, sphree, cylinder
  - Rotate, scale and translate each
  - extend scaling to limits, 0 or very large

2. Test 2: Check functionality of Adding, removing links
 - Delete links with childreen, base link
 - Delete duplicated links

3. Test 3: Select objects from tree or on scene
 - Ensure accuracy of parameters
 - Ensure item is selected on tree and on scene
 - Change all parameters, check clicking off and back on object
 
4. Test 4: Test Parameters
- Change to all potential types of sensors including sensor type, joint, 
- chagne joint origin and 

5. Test 5: Test URDF export
-Create acomplicated robot with mutliple link childreen, changed joint origin and position, and axis angle
- Export URDF description pacakge and simulation pacakge
- Follow instructuions in tutorials to build
- Check visually functionality 
- test joint accuracy
- check all simulation scenes work correctly

6. Test 6: Test imports and Exports
-Export complicated robot as GLTF and import and check functionailiy
-Import all sensors and check parameters are correct
- Import public repo of robots
- Check onboarding modal for functioanlity

7. Test 7: Check redo undo
- Create a multi link system, select undo and redo after multiple actions of changing values and positions and orientation of links.