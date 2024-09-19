import { useState, useRef } from 'react';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { faBars } from '@fortawesome/free-solid-svg-icons';
import MenuModal from '../../FunctionalComponents/MenuModal';

type Props = {
    openProjectManager: () => void,
    openImportDisplayer: () => void,
    openExportDisplayer: () => void,
}

const MenuIcon: React.FC<Props> = ({ openProjectManager, openImportDisplayer, openExportDisplayer }) => {
    const [isModalOpen, setIsModalOpen] = useState(false);
    const buttonRef = useRef<HTMLButtonElement>(null);

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

    const handleClick = () => {
        if(isModalOpen){
            setIsModalOpen(false);
        }else{
            setIsModalOpen(true);
        }
    };

    const closeModal = () => {
        setIsModalOpen(false);
    }

    return (
        <>
            <button className="menu-icon-button" onClick={handleClick} ref={buttonRef}>
                <FontAwesomeIcon icon={faBars} />
            </button>
            {isModalOpen && <MenuModal
            onClose={closeModal}
            menuItems={menuItems}
            buttonRef={buttonRef}
            />}
        </>
    );
};

export default MenuIcon;