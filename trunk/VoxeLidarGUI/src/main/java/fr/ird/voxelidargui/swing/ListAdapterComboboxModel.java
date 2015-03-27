/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package fr.ird.voxelidargui.swing;

import javax.swing.ComboBoxModel;
import javax.swing.DefaultComboBoxModel;
import javax.swing.ListModel;
import javax.swing.event.EventListenerList;
import javax.swing.event.ListDataEvent;
import javax.swing.event.ListDataListener;

/**
 *
 * @author Julien
 */
public class ListAdapterComboboxModel implements ComboBoxModel {

    private final DefaultComboBoxModel dataModel;
    private final DefaultComboBoxModel valueModel;
    private Object selectedObject;
    private final DataModelListDataListenerAdapter listDataListenerAdapter;

    public ListAdapterComboboxModel(DefaultComboBoxModel dataModel, DefaultComboBoxModel valueModel) {
        
        this.dataModel = dataModel;
        this.valueModel = valueModel;
        this.listDataListenerAdapter = new DataModelListDataListenerAdapter();
        dataModel.addListDataListener(listDataListenerAdapter);
    }
    
    public void addItemToModel(String data, String value){
        dataModel.addElement(data);
        valueModel.addElement(value);
    }
    
    public String getValue(int index){
        return valueModel.getElementAt(index).toString();
    }

    @Override
    public int getSize() {
        return dataModel.getSize();
    }

    @Override
    public Object getElementAt(int index) {
        return dataModel.getElementAt(index);
    }

    @Override
    public void addListDataListener(ListDataListener l) {
        listDataListenerAdapter.addListDataListener(l);
    }

    @Override
    public void removeListDataListener(ListDataListener l) {
        listDataListenerAdapter.removeListDataListener(l);
    }

    @Override
    public void setSelectedItem(Object anObject) {
        if ((selectedObject != null && !selectedObject.equals(anObject))
                || selectedObject == null && anObject != null) {
            selectedObject = anObject;
            ListDataEvent e = new ListDataEvent(this,
                    ListDataEvent.CONTENTS_CHANGED, -1, -1);
            listDataListenerAdapter.delegateListDataEvent(e);
        }
    }
    

    @Override
    public Object getSelectedItem() {
        return selectedObject;
    }

    private class DataModelListDataListenerAdapter implements ListDataListener {

        protected EventListenerList listenerList = new EventListenerList();

        public void removeListDataListener(ListDataListener l) {
            listenerList.remove(ListDataListener.class, l);
        }

        public void addListDataListener(ListDataListener l) {
            listenerList.add(ListDataListener.class, l);

        }

        @Override
        public void intervalAdded(ListDataEvent e) {
            delegateListDataEvent(e);
        }

        @Override
        public void intervalRemoved(ListDataEvent e) {
            checkSelection(e);
            delegateListDataEvent(e);
        }

        @Override
        public void contentsChanged(ListDataEvent e) {
            checkSelection(e);
            delegateListDataEvent(e);
        }

        private void checkSelection(ListDataEvent e) {
            Object selectedItem = getSelectedItem();
            ListModel listModel = (ListModel) e.getSource();
            int size = listModel.getSize();
            boolean selectedItemNoLongerExists = true;
            for (int i = 0; i < size; i++) {
                Object elementAt = listModel.getElementAt(i);
                if (elementAt != null && elementAt.equals(selectedItem)) {
                    selectedItemNoLongerExists = false;
                    break;
                }
            }
            if (selectedItemNoLongerExists) {
                ListAdapterComboboxModel.this.selectedObject = null;
            }
        }

        protected void delegateListDataEvent(ListDataEvent lde) {
            ListDataListener[] listeners = listenerList
                    .getListeners(ListDataListener.class);
            for (ListDataListener listDataListener : listeners) {
                listDataListener.contentsChanged(lde);
            }
        }

    }

}