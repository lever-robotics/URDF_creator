import { useState, useRef } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import MenuModal from '../../FunctionalComponents/MenuModal';

const MenuIcon = ({ openProjectManager, openImportDisplayer, openExportDisplayer }) => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const buttonRef = useRef(null);

    const menuItems = [
        { label: "Project Manager", action: () => {
            handleClick();
            openProjectManager();
        } },
        { label: "Import", action: () => {
            handleClick();
            openImportDisplayer();
        } },
        { label: "Export", action: () => {
            handleClick();
            openExportDisplayer();
        } },
        // { label: "Settings", action: () => alert("Setting") }
    ];

    const handleClick = (e) => {
        if(isModalOpen){
            setIsModalOpen(false);
        }else{
            setIsModalOpen(true);
        }
    };

    const closeModal = (e) => {
        if ((buttonRef.current && !buttonRef.current.contains(e.target))) {
            setIsModalOpen(false);
        }
    }

    return (
        <>
            <button className="menu-icon-button" onClick={handleClick} ref={buttonRef}>
                <FontAwesomeIcon icon={faBars} />
            </button>
            {isModalOpen && <MenuModal
            isOpen={isModalOpen}
            onClose={closeModal}
            menuItems={menuItems}
            buttonRef={buttonRef}
            />}
        </>
    );
};

export default MenuIcon;