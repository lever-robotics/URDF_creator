import type React from "react";

type EventFunctions = (event: PointerEvent) => void;

export class Mouse {
    mountDiv: HTMLDivElement;
    previousUpTime: number;
    currentDownTime: number;
    startPos: [number, number];
    onClickFunctions: EventFunctions[];
    onDoubleClickFunctions: EventFunctions[];
    x: number;
    y: number;

    constructor(mountDiv: HTMLDivElement) {
        this.mountDiv = mountDiv;
        this.previousUpTime = 0;
        this.currentDownTime = 0;
        this.startPos = [0, 0];

        this.onClickFunctions = [];
        this.onDoubleClickFunctions = [];
        this.x = 0;
        this.y = 0;
    }

    addListeners() {
        this.mountDiv.addEventListener(
            "pointerdown",
            this.onMouseDown.bind(this),
        );
        this.mountDiv.addEventListener("pointerup", this.onMouseUp.bind(this));
    }

    removeListeners() {
        this.mountDiv.removeEventListener(
            "pointerdown",
            this.onMouseDown.bind(this),
        );
        this.mountDiv.removeEventListener(
            "pointerup",
            this.onMouseUp.bind(this),
        );
    }

    onMouseDown(event: PointerEvent) {
        if ((event.target as HTMLElement).localName !== "canvas") return;
        console.log(this);

        event.preventDefault();
        this.currentDownTime = Date.now();
        this.startPos = [event.clientX, event.clientY];
    }

    onMouseUp(event: PointerEvent) {
        if ((event.target as HTMLElement).localName !== "canvas") return;
        console.log("onmouseup");
        event.preventDefault();
        const clickTime = 300;
        const dragThreshold = 20;
        const endPos = [event.clientX, event.clientY];

        // if the user clicks off of the three scene then drags in, return
        if (!this.startPos) return;

        if (
            Math.sqrt(
                (endPos[0] - this.startPos[0]) ** 2 +
                    (endPos[1] - this.startPos[1]) ** 2,
            ) > dragThreshold
        ) {
            // Do nothing if dragged
        } else if (
            this.currentDownTime - this.previousUpTime < clickTime &&
            Date.now() - this.currentDownTime < clickTime
        ) {
            this.onDoubleClick(event);
        } else if (Date.now() - this.currentDownTime < clickTime) {
            this.onClick(event);
        }
        this.previousUpTime = Date.now();
    }

    onDoubleClick(event: PointerEvent) {
        for (const func of this.onDoubleClickFunctions) {
            func(event);
        }
    }

    addOnDoubleClickFunctions(func: (event: PointerEvent) => void) {
        this.onDoubleClickFunctions.push(func);
    }

    addOnClickFunctions(func: (event: PointerEvent) => void) {
        this.onClickFunctions.push(func);
    }

    onClick(event: PointerEvent | PointerEvent) {
        for (const func of this.onClickFunctions) {
            func(event);
        }
    }
}
