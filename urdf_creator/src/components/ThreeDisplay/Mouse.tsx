import React from "react"

export class Mouse {

    mount: HTMLDivElement;
    eventFunctions: { type: string; func: (event: any) => void; }[];
    previousUpTime: number;
    currentDownTime: number;
    startPos: [number, number];
    onClickFunctions: Function[];
    onDoubleClickFunctions: Function[];
    x: number;
    y: number;

    constructor(mount: HTMLDivElement) {
        this.mount = mount;
        this.eventFunctions = [
            { type: "pointerdown", func: this.onMouseDown.bind(this) },
            { type: "pointerup", func: this.onMouseUp.bind(this) },
        ];
        this.previousUpTime = 0;
        this.currentDownTime = 0;
        this.startPos = [0, 0];

        this.onClickFunctions = [];
        this.onDoubleClickFunctions = [];
        this.x = 0;
        this.y = 0;
    }

    addListeners() {
        if (this.mount) {
            this.eventFunctions.forEach((event) => {
                this.mount.addEventListener(event.type, event.func);
            });
        }
    }

    callback() {
        if (this) {
            if (this.mount) {
                this.eventFunctions.forEach((event) => {
                    this.mount.removeEventListener(event.type, event.func);
                });
            }
        }
    }

    onMouseDown(event: React.MouseEvent<HTMLDivElement>) {
        if (event.currentTarget.className !== "display") return;
        event.preventDefault();
        this.currentDownTime = Date.now();
        this.startPos = [event.clientX, event.clientY];
    }

    onMouseUp(event: React.MouseEvent<HTMLDivElement>) {
        if (event.currentTarget.localName !== "display") return;
        event.preventDefault();
        const clickTime = 300;
        const dragThreshold = 20;
        const endPos = [event.clientX, event.clientY];

        // if the user clicks off of the three scene then drags in, return
        if (!this.startPos) return;

        if (Math.sqrt((endPos[0] - this.startPos[0]) ** 2 + (endPos[1] - this.startPos[1]) ** 2) > dragThreshold) {
            // Do nothing if dragged
        } else if (this.currentDownTime - this.previousUpTime < clickTime && Date.now() - this.currentDownTime < clickTime) {
            this.onDoubleClick(event);
        } else if (Date.now() - this.currentDownTime < clickTime) {
            this.onClick(event);
        }
        this.previousUpTime = Date.now();
    }

    onDoubleClick(event: React.MouseEvent<HTMLDivElement>) {
        this.onDoubleClickFunctions.forEach((func) => {
            func(event);
        });
    }

    addOnDoubleClickFunctions(func: (event: MouseEvent) => void) {
        this.onDoubleClickFunctions.push(func);
    }

    addOnClickFunctions(func: (event: MouseEvent) => void) {
        this.onClickFunctions.push(func);
    }

    onClick(event: React.MouseEvent<HTMLDivElement>) {
        this.onClickFunctions.forEach((func) => {
            func(event);
        });
    }
}
