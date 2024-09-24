import { Color, MeshPhongMaterial } from "three";
import Inertia from "./Inertia";
import { Sensor } from "./SensorsClass";

export default class UserData {
    shape: string;
    inertia: Inertia;
    isRootFrame: boolean;
    selectable: boolean;
    sensor?: Sensor;
    stlfile?: string;
    material?: MeshPhongMaterial;
    color?: Color;
    name: string;

    constructor(shape: string, name: string) {
        this.name = name;
        this.shape = shape;
        this.inertia = new Inertia();
        this.isRootFrame = false;
        this.selectable = true;
    }

    duplicate() {
        const duplicated = new UserData(this.shape, this.name);
        duplicated.stlfile = this.stlfile;
        duplicated.name = this.name + " copy";
        duplicated.inertia = this.inertia.duplicate();
        duplicated.isRootFrame = false;
        if (this.sensor) {
            duplicated.sensor = this.sensor.duplicate();
        }

        return duplicated;
    }
}
