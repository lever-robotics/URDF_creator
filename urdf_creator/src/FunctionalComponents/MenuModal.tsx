import type React from "react";
import { useRef } from "react";
import MenuItem from "./MenuItem";
import styles from "./MenuModal.module.css";
import useClickOutside from "./useClickOutside";

type Props = {
    onClose: () => void;
    menuItems: {
        label: string;
        id: number;
        action: () => void;
    }[];
    buttonRef: React.RefObject<HTMLButtonElement>;
};

const MenuModal: React.FC<Props> = ({ onClose, menuItems, buttonRef }) => {
    const wrapperRef = useRef<HTMLDivElement | null>(null);
    useClickOutside(wrapperRef, buttonRef, onClose);
    if (!buttonRef.current) return;
    const buttonRect = buttonRef.current.getBoundingClientRect();
    const position = {
        top: buttonRect.bottom + window.scrollY, // Position below button
        left: buttonRect.left + window.scrollX, // Align left with the button
    };

    return (
        <div
            className={styles.menuModal}
            ref={wrapperRef}
            style={{ top: position.top, left: position.left }}
        >
            <ul className={styles.menuList}>
                {menuItems.map((item, index) => (
                    <MenuItem
                        key={item.id}
                        action={item.action}
                        label={item.label}
                    />
                ))}
            </ul>
        </div>
    );
};

export default MenuModal;
