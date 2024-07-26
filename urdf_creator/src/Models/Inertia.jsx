export default class Inertia {
    constructor(mass = 1, ixx = 0, iyy = 0, izz = 0, ixy = 0, ixz = 0, iyz = 0) {
        this.customInertia = false;
        this.mass = mass;
        this.ixx = ixx;
        this.iyy = iyy;
        this.izz = izz;
        this.ixy = ixy;
        this.ixz = ixz;
        this.iyz = iyz;
    }

    // call every time the scaleing of the shape is changed
    updateInertia(threeObject) {
        if (this.customInertia) return;

        const shape = threeObject.shape;
        if (shape === 'cube') {
            const width = threeObject.link.scale.x;
            const height = threeObject.link.scale.y;
            const depth = threeObject.link.scale.z;
            this.ixx = (1 / 12) * this.mass * (height ** 2 + depth ** 2);
            this.iyy = (1 / 12) * this.mass * (width ** 2 + depth ** 2);
            this.izz = (1 / 12) * this.mass * (width ** 2 + height ** 2);
        } else if (shape === 'cylinder') {
            const radius = threeObject.link.scale.x;
            const height = threeObject.link.scale.y;
            this.ixx = (1 / 12) * this.mass * (3 * radius ** 2 + height ** 2);
            this.iyy = (1 / 2) * this.mass * radius ** 2;
            this.izz = this.ixx;
        } else if (shape === 'sphere') {
            const radius = threeObject.link.scale.x;
            this.ixx = (2 / 5) * this.mass * radius ** 2;
            this.iyy = this.ixx;
            this.izz = this.ixx;
        } else {
            console.error('Invalid shape input');
        }
    }

    setCustomInertia(ixx, iyy, izz, ixy, ixz, iyz) {
        this.customInertia = true;
        this.ixx = ixx;
        this.iyy = iyy;
        this.izz = izz;
        this.ixy = ixy;
        this.ixz = ixz;
        this.iyz = iyz;
    }

    updateMass(newMass, object) {
        this.mass = newMass;
        if (!this.customInertia) {
            this.updateInertia(object);
        }
    }

    duplicate() {
        const duplicated = new Inertia(this.mass, this.ixx, this.iyy, this.izz, this.ixy, this.ixz, this.iyz);
        duplicated.customInertia = this.customInertia;
        return duplicated;
    }
}