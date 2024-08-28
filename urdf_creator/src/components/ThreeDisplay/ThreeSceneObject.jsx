export class ThreeScene {
    constructor(mountRef, scene, camera, renderer, orbitControls, transformControls, lights, gridHelper, font, axesHelper, renderPass, composer, background, raycaster, mouse, callback) {
        this.mountRef = mountRef;
        this.scene = scene;
        this.camera = camera;
        this.renderer = renderer;
        this.orbitControls = orbitControls;
        this.transformControls = transformControls;
        this.lights = lights;
        this.gridHelper = gridHelper;
        this.font = font;
        this.axesHelper = axesHelper;
        this.renderPass = renderPass;
        this.composer = composer;
        this.background = background;
        this.raycaster = raycaster;
        this.mouse = mouse;
        this.callback = callback;
        this.baseLink = null;
    }
    addToScene(objects) {
        objects.forEach((object) => {
            this.scene.add(object);
        });
    }
}
