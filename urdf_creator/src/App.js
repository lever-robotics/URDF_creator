import React, { useRef, useEffect, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import { TransformControls } from 'three/examples/jsm/controls/TransformControls';

function App() {
  const mountRef = useRef(null);
  const [scene] = useState(new THREE.Scene());
  const [selectedObject, setSelectedObject] = useState(null);
  const [selectedShape, setSelectedShape] = useState("cube");

  useEffect(() => {
    const currentRef = mountRef.current;
    scene.background = new THREE.Color(0xf0f0f0);
  
    // Camera
    const camera = new THREE.PerspectiveCamera(75, currentRef.clientWidth / currentRef.clientHeight, 0.1, 1000);
    camera.position.set(5, 5, 5);
    camera.lookAt(new THREE.Vector3(0, 0, 0));
  
    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(currentRef.clientWidth, currentRef.clientHeight);
    currentRef.appendChild(renderer.domElement);
  
    // Orbit Controls
    const orbitControls = new OrbitControls(camera, renderer.domElement);
    orbitControls.enableDamping = true;
    orbitControls.dampingFactor = 0.1;
  
    // Transform Controls
    const transformControls = new TransformControls(camera, renderer.domElement);
    scene.add(transformControls);
    transformControls.addEventListener('dragging-changed', function (event) {
      orbitControls.enabled = !event.value; // Disable orbit controls when using transform controls
    });
  
    // Grid
    const gridHelper = new THREE.GridHelper(10, 10);
    scene.add(gridHelper);
  
    // Ambient Light
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
  
    // Raycaster for selection
    const raycaster = new THREE.Raycaster();
    const mouse = new THREE.Vector2();

    function addShape(position) {
      let geometry;
      switch(selectedShape) {
        case "sphere":
          geometry = new THREE.SphereGeometry(0.5, 32, 32);
          break;
        case "cylinder":
          geometry = new THREE.CylinderGeometry(0.5, 0.5, 1, 32);
          break;
        default: // "cube"
          geometry = new THREE.BoxGeometry(1, 1, 1);
      }
      const material = new THREE.MeshBasicMaterial({ color: Math.random() * 0xffffff });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.position.copy(position).add(new THREE.Vector3(0, 0.5, 0)); // Adjust position to place on the grid
      scene.add(mesh);
    }
  
    function onSelectObject(event) {
      mouse.x = (event.clientX / window.innerWidth) * 2 - 1;
      mouse.y = - (event.clientY / window.innerHeight) * 2 + 1;
      raycaster.setFromCamera(mouse, camera);
      const intersects = raycaster.intersectObjects(scene.children, true);
      
      if (intersects.length > 0 && intersects[0].object !== gridHelper) {
        const selected = intersects[0].object;
        setSelectedObject(selected);
        transformControls.attach(selected);
      } else {
        transformControls.detach();
        setSelectedObject(null);
      }
    }
  
    window.addEventListener('pointerdown', onSelectObject);
  
    // Animation loop
    const animate = function () {
      requestAnimationFrame(animate);
      orbitControls.update();
      renderer.render(scene, camera);
    };
    animate();
  
    // Cleanup
    return () => {
      currentRef.removeChild(renderer.domElement);
      orbitControls.dispose();
      transformControls.dispose();
      window.removeEventListener('pointerdown', onSelectObject);
    };
  }, [scene, selectedShape]);
  

  return (
    <div style={{ width: '100vw', height: '100vh', display: 'flex' }}>
      <div ref={mountRef} style={{ flexGrow: 1 }} />
      <div style={{ display: 'flex', flexDirection: 'column', padding: 10 }}>
        <button onClick={() => setSelectedObject(null)}>Deselect</button>
        <div style={{ display: 'flex', flexDirection: 'column', padding: 10 }}>
        <button onClick={() => setSelectedShape("cube")}>Cube</button>
        <button onClick={() => setSelectedShape("sphere")}>Sphere</button>
        <button onClick={() => setSelectedShape("cylinder")}>Cylinder</button>
      </div>
      </div>
    </div>
  );
}

export default App;
