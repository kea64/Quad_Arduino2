#ifndef Linked_List_h
#define Linked_List_h

template<typename T>
class Node {
  public:
    //Constructors
    Node();
    Node(T d, Node<T> * l = NULL);
    
    //Inspectors
    T data() const;
    Node<T> * link() const;
    
    //Mutators
    void data(T d); //assign new data to node
    void link(Node<T> * l);
    
    //Destructor
    ~Node();
  
  private:
  
    Node<T> * link_;
    T data_;
};

template<typename T>
Node<T>::Node(){
  link_ = NULL;
  data_ = T();
}

template<typename T>
Node<T>::Node(T d, Node<T> * l){
  link_ = l;
  data_ = d;
}

template<typename T>
Node<T>::~Node(){
  if (link_!= NULL){delete link_;}
}

template<typename T>
T Node<T>::data() const{
  return(data_);
}

template<typename T>
Node<T>* Node<T>::link() const{
  return(link_);
}

template<typename T>
void Node<T>::data(T d){
  data_ = d;
}

template<typename T>
void Node<T>::link(Node<T> * l){
  link_ = l;
}

#endif 
