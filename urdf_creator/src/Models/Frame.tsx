import * as THREE from "three";
import { Euler, Vector3 } from "three";
import { registerName } from "../components/ThreeDisplay/TreeUtils";
import type Axis from "./Axis";
import type Inertia from "./Inertia";
import type JointVisualizer from "./JointVisualizer";
import type Link from "./Link";
import { Camera, IMU, Lidar, Sensor } from "./SensorsClass";
import { Collision, Visual } from "./VisualCollision";
import type VisualCollision from "./VisualCollision";

export type Frameish = Frame | null | undefined;
export type JointType = "fixed" | "revolute" | "continuous" | "prismatic";
export type Shape = "cube" | "sphere" | "cylinder" | "mesh";

export default class Frame extends THREE.Mesh {
    visuals!: Visual[];
    collisions!: Collision[];
    link!: Link;
    sensor!: Sensor;
    inertia!: Inertia;
    jointVisualizer!: JointVisualizer;
    axis!: Axis;
    parentFrame!: Frame;

    isRootFrame: boolean;
    isWorldFrame: boolean;

    stlfile?: string;
    _jointType: JointType;

    constructor(
        public name = "",
        position: Vector3 = new Vector3(0, 0, 0),
        rotation: Euler = new Euler(0, 0, 0),
        jointType: JointType = "fixed",
        public min = -1,
        public max = 1,
    ) {
        const matWhite = new THREE.MeshBasicMaterial({
            depthTest: false,
            depthWrite: false,
            fog: false,
            toneMapped: false,
            transparent: true,
        });
        matWhite.opacity = 0.15;
        super(new THREE.OctahedronGeometry(0.075, 0), matWhite);

        this.name = name;

        this.position.copy(position);
        this.rotation.copy(rotation);

        this.isRootFrame = false;
        this.isWorldFrame = false;

        this._jointType = jointType;
    }

    /**
     *
     *
     *
     *
     * GETTER/SETTER: Alphabetical
     *
     *
     *
     *
     **/

    getFrameChildren = () => {
        return this.jointVisualizer.children.filter(
            (child) => child instanceof Frame,
        );
    };

    get parentName() {
        return this.parentFrame.name;
    }

    get objectVisuals() {
        return this.visuals;
    }

    get propertyNames() {
        const properties = this.visuals.concat(this.collisions);
        return properties.map((property) => property.name);
    }

    get objectCollisions() {
        return this.collisions;
    }

    get jointType() {
        return this._jointType;
    }

    set jointType(type) {
        this._jointType = type;
        switch (type) {
            case "fixed":
                this.axis.material.visible = false;
                break;
            case "revolute":
            case "prismatic":
                this.min = -1;
                this.max = 1;
                this.axis.material.visible = true;
                break;
            case "continuous":
                this.min = -3.14;
                this.max = 3.14;
                this.axis.material.visible = true;
                break;
            default:
                break;
        }
    }

    get offset() {
        return this.link.position;
    }

    set mass(mass) {
        this.inertia.updateMass(mass, this);
    }

    get mass() {
        return this.inertia.mass;
    }

    get axisRotation() {
        return this.axis.rotation;
    }

    set axisRotation(rotation: Euler) {
        this.axis.rotation.copy(rotation);
    }

    attachChild(child: Frame) {
        if (!this.jointVisualizer) return;
        this.jointVisualizer.attach(child);
        child.parentFrame = this;
    }

    addChild(child: Frame) {
        if (!this.jointVisualizer) return;
        this.jointVisualizer.add(child);
        child.parentFrame = this;
    }

    get sensorType() {
        return this.sensor.type ?? "";
    }

    set sensorType(type) {
        switch (type) {
            case "imu":
                this.sensor = new IMU();
                break;
            case "camera":
                this.sensor = new Camera();
                break;
            case "lidar":
                this.sensor = new Lidar();
                break;
            case "":
                this.sensor = new Sensor();
                break;
            // Add cases for other sensor types here
            default:
                throw Error("This type of sensor is not yet supported");
        }
    }

    updateInertia() {
        if (!this.inertia) return;
        this.inertia.updateInertia(this);
    }

    /**
     * Add a VisualCollision property to the Frame
     * @param VisualCollision
     */
    addProperty(property: Visual | Collision) {
        if (property instanceof Visual) {
            this.visuals.push(property);
        } else if (property instanceof Collision) {
            this.collisions.push(property);
        }
        property.name = registerName(property.name, this.propertyNames);
        property.frame = this;
        this.link.add(property);
    }

    removeProperty(property: VisualCollision) {
        if (property instanceof Visual) {
            const index = this.visuals.indexOf(property);
            this.visuals.splice(index, 1); // Take the property out of the visuals array
            property.removeFromParent();
            return;
        }
        if (property instanceof Collision) {
            const index = this.collisions.indexOf(property);
            this.collisions.splice(index, 1); // Take the property out of the collisions array
            property.removeFromParent();
            return;
        }
    }

    rotateAroundJointAxis = (angle: number) => {
        if (!this.axis || !this.jointVisualizer) return;
        // Angle must be in radians
        // a quaternion is basically how to get from one rotation to another
        const quaternion = new THREE.Quaternion();

        // this function calculates how to get from <0, 0, 0> (no rotation), to whatever the axis is currently rotated to in quaternions
        quaternion.setFromEuler(this.axis.rotation);

        // the joint axis is always set to <1, 0, 0>, but it rotates around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);

        // the joint's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer.setRotationFromAxisAngle(newAxis, angle);
    };

    translateAlongJointAxis = (distance: number) => {
        const quaternion = new THREE.Quaternion();
        // a quaternion is basically how to get from one rotation to another
        // this function says how to get from <0, 0, 0> (no rotation), to whatever the joint axis is currently rotated to
        quaternion.setFromEuler(this.axis.rotation);
        // the joint axis is always set to <1, 0, 0>, but it still moves around as the user rotates it
        // this function looks at the rotation of the axis and calculates what it would be if it was visually the same but rotation is set to <0, 0, 0>
        const newAxis = new THREE.Vector3(0, 0, 1).applyQuaternion(quaternion);
        // the shimmy's rotation is then set to be a rotation around the new axis by this angle
        this.jointVisualizer.position.set(0, 0, 0);
        this.jointVisualizer.translateOnAxis(newAxis, distance);
    };

    duplicate() {
        return new Frame(
            this.name,
            this.position,
            this.rotation,
            this.jointType,
            this.min,
            this.max,
        );
    }

    //Add STL to the Frame
    // setSTL = (stlfile) => {
    //     const loader = new STLLoader();
    //     loader.load(stlfile, (geometry) => {
    //         const material = new THREE.MeshPhongMaterial({ color: Math.random() * 0xffffff });
    //         const mesh = new THREE.Mesh(geometry, material);
    //         this.shimmy.link.mesh.add(mesh);
    //     });
    //     //make the mesh object a wireframe
    //     this.shimmy.link.mesh.children[0].material.wireframe = true;
    // }
}
