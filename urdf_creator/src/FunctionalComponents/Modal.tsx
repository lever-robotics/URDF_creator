// Modal.js
import React, { useEffect, useRef, useState } from "react";
import "./Modal.css";

type Props = {
    isOpen: boolean;
    onClose: () => void;
    modalContent: JSX.Element;
};

const Modal = ({ isOpen, onClose, modalContent }: Props) => {
    const [isVisible, setIsVisible] = useState(false);
    const overlayRef = useRef<HTMLDivElement>(null);
    const contentRef = useRef<HTMLDivElement>(null);

    useEffect(() => {
        if (isOpen) {
            setIsVisible(true);
        } else {
            setIsVisible(false);
        }
    }, [isOpen]);

    useEffect(() => {
        if (isVisible) {
            const overlay = overlayRef.current;
            const content = contentRef.current;

            if (overlay && content) {
                requestAnimationFrame(() => {
                    overlay.classList.add("open");
                    content.classList.add("open");
                });
            }
        } else {
            const overlay = overlayRef.current;
            const content = contentRef.current;

            if (overlay && content) {
                overlay.classList.remove("open");
                content.classList.remove("open");
            }
        }
    }, [isVisible]);

    const handleClose = () => {
        const overlay = overlayRef.current;
        const content = contentRef.current;

        if (overlay && content) {
            overlay.classList.remove("open");
            content.classList.remove("open");
            setTimeout(() => setIsVisible(false), 300); // Match the duration of the CSS transition
            onClose();
        }
    };

    if (!isVisible) return null;

    return (
        <div className="modal-overlay" onClick={handleClose} ref={overlayRef}>
            <div
                className="modal-content"
                onClick={(e) => e.stopPropagation()}
                ref={contentRef}
            >
                <button
                    className="close-button"
                    onClick={handleClose}
                    type={"button"}
                >
                    &times;
                </button>
                <div className="modal-body">{modalContent}</div>
            </div>
        </div>
    );
};

export default Modal;
